#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ICM-42688-P 测试板调试程序（Linux + python-periphery）。

功能包括：SPI身份验证、六轴及温度数据读取、INT1数据就绪中断、
PWM开环加热，以及利用芯片内部温度传感器进行比例闭环恒温控制。

硬件假设：
  * 鲁班猫SPI与ICM的VDDIO均为3.3 V，使用四线SPI直连。
  * 加热MOS管高电平导通，由指定的pwmchip/channel驱动。
  * 必须先通过id和stream测试，再开启任何加热功能。
"""

import argparse
import math
import signal
import struct
import sys
import time

from periphery import GPIO, PWM, SPI


# 以下均为本程序使用的Bank 0寄存器地址。
WHO_AM_I = 0x75          # 芯片身份寄存器，ICM-42688-P固定返回0x47
WHO_AM_I_EXPECTED = 0x47
DEVICE_CONFIG = 0x11     # 软件复位及SPI模式选择
TEMP_DATA1 = 0x1D        # 温度数据高字节，也是连续样本读取的起始地址
PWR_MGMT0 = 0x4E        # 温度、陀螺仪和加速度计电源模式
GYRO_CONFIG0 = 0x4F     # 陀螺仪量程与输出数据率
ACCEL_CONFIG0 = 0x50    # 加速度计量程与输出数据率
GYRO_CONFIG1 = 0x51     # 陀螺仪及温度滤波配置
INT_CONFIG = 0x14       # INT1/INT2输出模式、驱动方式及极性
INT_CONFIG1 = 0x64      # 中断异步复位等配置
INT_SOURCE0 = 0x65      # INT1事件映射
REG_BANK_SEL = 0x76     # 寄存器Bank选择

TEMP_INVALID = -32768    # 0x8000表示传感器样本尚未有效


class ICM42688:
    """ICM-42688-P四线SPI寄存器访问封装。"""

    def __init__(self, device: str, speed: int):
        # 参数依次是spidev设备、SPI模式、时钟频率。
        # ICM上电默认支持Mode 0和Mode 3，这里使用Mode 0。
        self.spi = SPI(device, 0, speed)
        # Allow the sensor supply and internal oscillator to settle after the
        # Linux process opens the SPI controller.
        time.sleep(0.010)

    def close(self):
        """关闭Linux spidev设备。"""
        self.spi.close()

    def read(self, register: int, length: int = 1) -> bytes:
        """从指定寄存器开始连续读取length个字节。

        地址字节bit7=1表示读操作，bit6:0为寄存器地址。
        例如读取WHO_AM_I(0x75)时发送F5 00，正常接收xx 47。
        地址和空字节放在一次transfer中，可保证整个事务期间CS保持低。
        """
        tx = [register | 0x80] + [0x00] * length
        rx = self.spi.transfer(tx)
        if len(rx) != len(tx):
            raise RuntimeError(
                f"short SPI transfer: sent {len(tx)} bytes, received {len(rx)}"
            )
        # Print the identity transaction only. This provides a useful hardware
        # diagnostic without flooding output during continuous sample reads.
        if register == WHO_AM_I:
            print(
                "SPI raw: "
                f"TX={[f'0x{value:02X}' for value in tx]} "
                f"RX={[f'0x{value:02X}' for value in rx]}"
            )
        # 首个RX字节与地址同时传输，不包含有效寄存器数据，因此丢弃。
        return bytes(rx[1:])

    def write(self, register: int, value: int):
        """写单个8位寄存器；bit7清零表示写操作。"""
        self.spi.transfer([register & 0x7F, value & 0xFF])

    def select_bank(self, bank: int):
        """选择寄存器Bank，只使用参数低3位。"""
        self.write(REG_BANK_SEL, bank & 0x07)

    def read_u8(self, register: int) -> int:
        """读取一个无符号8位寄存器值。"""
        return self.read(register, 1)[0]

    def initialize_200_hz(self):
        """复位并配置为±4 g、±250 dps、200 Hz六轴低噪声模式。"""
        self.select_bank(0)
        self.write(DEVICE_CONFIG, 0x01)  # soft reset
        time.sleep(0.010)
        self.select_bank(0)

        device_id = self.read_u8(WHO_AM_I)
        if device_id != WHO_AM_I_EXPECTED:
            raise RuntimeError(
                f"WHO_AM_I mismatch: got 0x{device_id:02X}, "
                f"expected 0x{WHO_AM_I_EXPECTED:02X}"
            )

        # ±250 dps, 200 Hz
        self.write(GYRO_CONFIG0, 0x67)
        # ±4 g, 200 Hz
        self.write(ACCEL_CONFIG0, 0x47)

        # Temperature DLPF = 10 Hz while preserving lower reset fields.
        gyro_config1 = self.read_u8(GYRO_CONFIG1)
        self.write(GYRO_CONFIG1, (gyro_config1 & 0x1F) | 0xA0)

        # TEMP enabled, gyro LN, accel LN.
        self.write(PWR_MGMT0, 0x0F)
        time.sleep(0.050)

    def configure_int1_data_ready(self):
        """把200 Hz用户接口数据就绪事件输出到INT1。"""
        self.select_bank(0)
        # INT1 pulse, push-pull, active high.
        self.write(INT_CONFIG, 0x03)
        # Required by datasheet for proper INT pin operation at ODR < 4 kHz.
        self.write(INT_CONFIG1, 0x00)
        # Route UI data-ready to INT1.
        self.write(INT_SOURCE0, 0x08)

    def read_sample(self):
        """读取一次温度、三轴加速度和三轴角速度。

        从TEMP_DATA1开始连续读取14字节：
        温度2字节 + 加速度XYZ共6字节 + 陀螺仪XYZ共6字节。
        七个通道均为大端、有符号、二进制补码16位数据。
        """
        # Temp(2) + accel XYZ(6) + gyro XYZ(6), big-endian register order.
        data = self.read(TEMP_DATA1, 14)
        values = struct.unpack(">hhhhhhh", data)
        temp_raw, ax, ay, az, gx, gy, gz = values
        if temp_raw == TEMP_INVALID:
            raise RuntimeError("temperature sample is invalid (0x8000)")

        temperature = temp_raw / 132.48 + 25.0
        accel = (ax / 8192.0, ay / 8192.0, az / 8192.0)  # ±4 g
        gyro = (gx / 131.0, gy / 131.0, gz / 131.0)  # ±250 dps
        return temperature, accel, gyro, temp_raw


class HeaterPWM:
    """Linux PWM加热输出封装，退出时保证加热关闭。"""

    def __init__(self, chip: int, channel: int, frequency: float):
        # 例如PWM(3, 0)对应pwmchip3的通道0。
        self.pwm = PWM(chip, channel)
        self.pwm.frequency = frequency
        # 先把占空比设为0，再使能PWM，避免启动瞬间意外加热。
        self.pwm.duty_cycle = 0.0
        self.pwm.enable()

    def set_duty(self, duty: float):
        """设置占空比，并强制限制在0.0～1.0范围内。"""
        self.pwm.duty_cycle = max(0.0, min(1.0, duty))

    def close(self):
        """先关闭加热，再释放PWM设备。"""
        try:
            self.pwm.duty_cycle = 0.0
            self.pwm.disable()
        finally:
            self.pwm.close()


class HeaterGPIO:
    """Digital GPIO heater output; LOW is the fail-safe OFF state."""

    def __init__(self, line: int, chip: str = None):
        # Newer python-periphery versions use the character-device form:
        # GPIO("/dev/gpiochipN", line_offset, "out").  With no chip argument,
        # retain compatibility with the older global-line-number API.
        if chip:
            self.gpio = GPIO(chip, line, "out")
        else:
            self.gpio = GPIO(line, "out")
        self.gpio.write(False)

    def set_enabled(self, enabled: bool):
        self.gpio.write(bool(enabled))

    def close(self):
        try:
            self.gpio.write(False)
        finally:
            self.gpio.close()


def check_temperature(value: float, hard_limit: float):
    """验证温度有效性，并在超出安全范围时终止加热。"""
    if not math.isfinite(value):
        raise RuntimeError("temperature is not finite")
    if value < -40.0 or value > hard_limit:
        raise RuntimeError(
            f"temperature safety stop: {value:.2f} °C "
            f"(allowed -40 to {hard_limit:.2f} °C)"
        )


def cmd_id(imu: ICM42688, _args):
    """读取WHO_AM_I；所有其他功能测试前应先通过此项。"""
    imu.select_bank(0)
    value = imu.read_u8(WHO_AM_I)
    print(f"WHO_AM_I = 0x{value:02X}")
    if value != WHO_AM_I_EXPECTED:
        raise RuntimeError(f"expected 0x{WHO_AM_I_EXPECTED:02X}")
    print("PASS: SPI communication and device identity")


def cmd_stream(imu: ICM42688, args):
    """连续输出芯片温度、加速度和角速度。"""
    imu.initialize_200_hz()
    end = time.monotonic() + args.duration
    count = 0
    while time.monotonic() < end:
        temp, accel, gyro, raw = imu.read_sample()
        print(
            f"T={temp:7.2f} °C raw={raw:6d} | "
            f"A=({accel[0]:+7.3f},{accel[1]:+7.3f},{accel[2]:+7.3f}) g | "
            f"G=({gyro[0]:+8.2f},{gyro[1]:+8.2f},{gyro[2]:+8.2f}) dps"
        )
        count += 1
        time.sleep(1.0 / args.rate)
    print(f"PASS: captured {count} samples")


def cmd_interrupt(imu: ICM42688, args):
    """配置INT1数据就绪输出，并留出示波器测量时间。"""
    imu.initialize_200_hz()
    imu.configure_int1_data_ready()
    print("INT1 configured: push-pull, active-high, data-ready, 200 Hz")
    print(f"Measure INT1 with an oscilloscope for {args.duration:.1f} seconds.")
    print("Expected result: approximately 200 pulses/second.")
    time.sleep(args.duration)


def cmd_heater_open_loop(imu: ICM42688, args):
    """使用固定PWM占空比开环加热，并持续执行过温保护。"""
    imu.initialize_200_hz()
    heater = HeaterPWM(args.pwmchip, args.channel, args.frequency)
    try:
        initial, _, _, _ = imu.read_sample()
        check_temperature(initial, args.hard_limit)
        print(f"Initial die temperature: {initial:.2f} °C")
        print(
            f"Applying {args.duty * 100:.1f}% duty at "
            f"{args.frequency:.0f} Hz for {args.duration:.1f} s"
        )
        heater.set_duty(args.duty)

        end = time.monotonic() + args.duration
        last = initial
        while time.monotonic() < end:
            last, _, _, _ = imu.read_sample()
            check_temperature(last, args.hard_limit)
            print(
                f"t_left={end - time.monotonic():6.1f}s "
                f"T={last:7.2f} °C ΔT={last - initial:+6.2f} °C"
            )
            time.sleep(0.5)
        print(f"PASS: temperature changed by {last - initial:+.2f} °C")
    finally:
        heater.close()
        print("Heater forced OFF")


def cmd_closed_loop(imu: ICM42688, args):
    """使用一阶温度滤波和比例控制实现闭环加热。

    误差 = 目标温度 - 滤波温度
    占空比 = kp × 误差，并限制在0到max_duty之间。
    本电路只能加热，温度高于目标值时只能关闭PWM，不能主动制冷。
    """
    if args.target >= args.hard_limit:
        raise ValueError("target must be below hard-limit")

    imu.initialize_200_hz()
    heater = HeaterPWM(args.pwmchip, args.channel, args.frequency)
    filtered = None
    stable_samples = 0
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            temp, _, _, _ = imu.read_sample()
            check_temperature(temp, args.hard_limit)
            # 指数低通滤波，降低温度测量噪声引起的PWM抖动。
            filtered = temp if filtered is None else 0.8 * filtered + 0.2 * temp

            # 简单比例控制器，并限制最大加热功率。
            error = args.target - filtered
            duty = max(0.0, min(args.max_duty, args.kp * error))
            heater.set_duty(duty)

            if abs(error) <= args.tolerance:
                stable_samples += 1
            else:
                stable_samples = 0

            print(
                f"T={temp:7.2f} °C filt={filtered:7.2f} °C "
                f"target={args.target:5.1f} °C duty={duty * 100:5.1f}% "
                f"error={error:+6.2f} °C"
            )
            time.sleep(0.1)

        if stable_samples >= 50:
            print("PASS: temperature remained in tolerance for at least 5 seconds")
        else:
            print("RESULT: test completed, but 5-second stability criterion was not met")
    finally:
        heater.close()
        print("Heater forced OFF")


def cmd_thermostat(imu: ICM42688, args):
    """Control the heater with two temperature thresholds (hysteresis).

    The heater turns on at or below ``low`` and turns off at or above
    ``high``.  Between the thresholds it keeps its previous state, which
    prevents rapid PWM on/off chatter caused by sensor noise.
    """
    if args.low >= args.high:
        raise ValueError("low must be below high")
    if args.high >= args.hard_limit:
        raise ValueError("high must be below hard-limit")
    if not 0.0 < args.duty <= 1.0:
        raise ValueError("duty must be greater than 0 and at most 1")
    if args.duration <= 0.0:
        raise ValueError("duration must be greater than 0")

    imu.initialize_200_hz()
    heater = HeaterPWM(args.pwmchip, args.channel, args.frequency)
    filtered = None
    heater_on = False
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            temp, _, _, _ = imu.read_sample()
            check_temperature(temp, args.hard_limit)
            filtered = temp if filtered is None else 0.8 * filtered + 0.2 * temp

            previous_state = heater_on
            if filtered <= args.low:
                heater_on = True
            elif filtered >= args.high:
                heater_on = False

            duty = args.duty if heater_on else 0.0
            heater.set_duty(duty)

            if heater_on != previous_state:
                print(
                    f"STATE CHANGE: heater {'ON' if heater_on else 'OFF'} "
                    f"at {filtered:.2f} degC"
                )

            print(
                f"T={temp:7.2f} degC filt={filtered:7.2f} degC "
                f"low={args.low:5.2f} high={args.high:5.2f} "
                f"heater={'ON ' if heater_on else 'OFF'} "
                f"duty={duty * 100:4.1f}%"
            )
            time.sleep(0.1)
    finally:
        heater.close()
        print("Heater forced OFF")


def cmd_gpio_thermostat(imu: ICM42688, args):
    """Bang-bang thermostat using a high-active digital GPIO output."""
    if args.low >= args.high:
        raise ValueError("low must be below high")
    if args.high >= args.hard_limit:
        raise ValueError("high must be below hard-limit")
    if args.duration <= 0.0:
        raise ValueError("duration must be greater than 0")

    imu.initialize_200_hz()
    heater = HeaterGPIO(args.line, args.gpiochip)
    filtered = None
    heater_on = False
    try:
        end = time.monotonic() + args.duration
        while time.monotonic() < end:
            temp, _, _, _ = imu.read_sample()
            check_temperature(temp, args.hard_limit)
            filtered = temp if filtered is None else 0.8 * filtered + 0.2 * temp

            previous_state = heater_on
            if filtered <= args.low:
                heater_on = True
            elif filtered >= args.high:
                heater_on = False

            heater.set_enabled(heater_on)
            if heater_on != previous_state:
                print(
                    f"STATE CHANGE: GPIO heater "
                    f"{'ON (1)' if heater_on else 'OFF (0)'} "
                    f"at {filtered:.2f} degC"
                )

            print(
                f"T={temp:7.2f} degC filt={filtered:7.2f} degC "
                f"low={args.low:5.2f} high={args.high:5.2f} "
                f"gpio={1 if heater_on else 0}"
            )
            time.sleep(0.1)
    finally:
        heater.close()
        print("GPIO heater forced OFF (0)")


def build_parser():
    """定义全局参数以及id、stream、interrupt、heater和closed-loop命令。"""
    parser = argparse.ArgumentParser()
    parser.add_argument("--spi", required=True, help="for example /dev/spidev1.0")
    parser.add_argument("--speed", type=int, default=1_000_000)
    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("id")

    stream = sub.add_parser("stream")
    stream.add_argument("--duration", type=float, default=10.0)
    stream.add_argument("--rate", type=float, default=10.0)

    interrupt = sub.add_parser("interrupt")
    interrupt.add_argument("--duration", type=float, default=10.0)

    def add_heater_options(command):
        # heater和closed-loop共用的PWM及安全参数。
        command.add_argument("--pwmchip", type=int, default=3)
        command.add_argument("--channel", type=int, default=0)
        command.add_argument("--frequency", type=float, default=1000.0)
        command.add_argument("--duration", type=float, default=30.0)
        command.add_argument("--hard-limit", type=float, default=50.0)

    heater = sub.add_parser("heater")
    add_heater_options(heater)
    heater.add_argument("--duty", type=float, default=0.10)

    closed = sub.add_parser("closed-loop")
    add_heater_options(closed)
    closed.set_defaults(duration=300.0)
    closed.add_argument("--target", type=float, default=35.0)
    closed.add_argument("--max-duty", type=float, default=0.50)
    closed.add_argument("--kp", type=float, default=0.10)
    closed.add_argument("--tolerance", type=float, default=0.5)

    thermostat = sub.add_parser(
        "thermostat",
        help="automatic heater on/off control with temperature hysteresis",
    )
    add_heater_options(thermostat)
    thermostat.set_defaults(duration=300.0)
    thermostat.add_argument("--low", type=float, default=26.7)
    thermostat.add_argument("--high", type=float, default=27.0)
    thermostat.add_argument("--duty", type=float, default=0.01)

    gpio_thermostat = sub.add_parser(
        "gpio-thermostat",
        help="automatic full-on/full-off GPIO heater control with hysteresis",
    )
    gpio_thermostat.add_argument(
        "--gpiochip",
        help="for example /dev/gpiochip1; omit for legacy global GPIO numbers",
    )
    gpio_thermostat.add_argument(
        "--line",
        type=int,
        required=True,
        help="GPIO line offset within gpiochip, or legacy global GPIO number",
    )
    gpio_thermostat.add_argument("--duration", type=float, default=300.0)
    gpio_thermostat.add_argument("--hard-limit", type=float, default=32.0)
    gpio_thermostat.add_argument("--low", type=float, default=26.5)
    gpio_thermostat.add_argument("--high", type=float, default=27.0)

    return parser


def main():
    """解析参数、打开SPI、执行子命令，并在退出时关闭SPI。"""
    args = build_parser().parse_args()
    imu = None
    try:
        imu = ICM42688(args.spi, args.speed)
        {
            "id": cmd_id,
            "stream": cmd_stream,
            "interrupt": cmd_interrupt,
            "heater": cmd_heater_open_loop,
            "closed-loop": cmd_closed_loop,
            "thermostat": cmd_thermostat,
            "gpio-thermostat": cmd_gpio_thermostat,
        }[args.command](imu, args)
    finally:
        if imu is not None:
            imu.close()


if __name__ == "__main__":
    # SIGTERM becomes a normal exception path so PWM cleanup can run.
    signal.signal(signal.SIGTERM, lambda _signum, _frame: sys.exit(143))
    main()
