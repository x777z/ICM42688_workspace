#!/usr/bin/env python3
"""Monitor ICM-42688 and a YLX-2UQ2 camera in internal 60 Hz mode.

The script leaves the camera's BLS control unchanged and configures only the
standard V4L2 video format and frame interval. No external PWM is generated.
"""

import argparse
import struct
import subprocess
import threading
import time
from pathlib import Path

import cv2
from periphery import SPI


def run_v4l2(device: str, *arguments: str) -> str:
    """Run v4l2-ctl and return its output, failing on rejected settings."""
    result = subprocess.run(
        ["v4l2-ctl", "-d", device, *arguments],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or "v4l2-ctl failed")
    return result.stdout.strip()


def configure_v4l2_mode(device: str, width: int, height: int, fps: float) -> None:
    """Set the real V4L2 format/frame interval before OpenCV opens the device."""
    run_v4l2(
        device,
        f"--set-fmt-video=width={width},height={height},pixelformat=MJPG",
        f"--set-parm={fps:g}",
    )


class ICM42688:
    WHO_AM_I = 0x75
    WHO_AM_I_EXPECTED = 0x47
    TEMP_DATA1 = 0x1D
    PWR_MGMT0 = 0x4E
    GYRO_CONFIG0 = 0x4F
    ACCEL_CONFIG0 = 0x50
    GYRO_CONFIG1 = 0x51
    REG_BANK_SEL = 0x76
    TEMP_INVALID = -32768

    def __init__(self, device: str, speed: int = 1_000_000):
        self.spi = SPI(device, 0, speed)
        time.sleep(0.010)

    def close(self):
        self.spi.close()

    def _read(self, register: int, length: int = 1) -> bytes:
        tx = [register | 0x80] + [0x00] * length
        rx = self.spi.transfer(tx)
        if len(rx) != len(tx):
            raise RuntimeError("SPI transfer short")
        return bytes(rx[1:])

    def _write(self, register: int, value: int):
        self.spi.transfer([register & 0x7F, value & 0xFF])

    def _select_bank(self, bank: int):
        self._write(self.REG_BANK_SEL, bank & 0x07)

    def _read_u8(self, register: int) -> int:
        return self._read(register, 1)[0]

    def initialize_200_hz(self):
        self._select_bank(0)
        self._write(0x11, 0x01)
        time.sleep(0.010)
        self._select_bank(0)

        whoami = self._read_u8(self.WHO_AM_I)
        if whoami != self.WHO_AM_I_EXPECTED:
            raise RuntimeError(
                f"WHO_AM_I=0x{whoami:02X}, expected 0x{self.WHO_AM_I_EXPECTED:02X}"
            )

        self._write(self.GYRO_CONFIG0, 0x67)
        self._write(self.ACCEL_CONFIG0, 0x47)
        gyro_conf1 = self._read_u8(self.GYRO_CONFIG1)
        self._write(self.GYRO_CONFIG1, (gyro_conf1 & 0x1F) | 0xA0)
        self._write(self.PWR_MGMT0, 0x0F)
        time.sleep(0.050)

    def read_sample(self):
        data = self._read(self.TEMP_DATA1, 14)
        temp_raw, ax, ay, az, gx, gy, gz = struct.unpack(">hhhhhhh", data)
        if temp_raw == self.TEMP_INVALID:
            raise RuntimeError("temperature sample invalid")
        temperature = temp_raw / 132.48 + 25.0
        accel = (ax / 8192.0, ay / 8192.0, az / 8192.0)
        gyro = (gx / 131.0, gy / 131.0, gz / 131.0)
        return temperature, accel, gyro, temp_raw


class CameraReader(threading.Thread):
    def __init__(self, cap: cv2.VideoCapture, raw_mjpeg: bool):
        super().__init__(daemon=True)
        self.cap = cap
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest_frame = None
        self.frame_count = 0
        self.read_errors = 0
        self.raw_mjpeg = raw_mjpeg

    def run(self) -> None:
        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                self.read_errors += 1
                time.sleep(0.001)
                continue
            with self.lock:
                self.latest_frame = frame
                self.frame_count += 1

    def get_latest_frame(self):
        with self.lock:
            return None if self.latest_frame is None else self.latest_frame.copy()

    def get_counters(self):
        with self.lock:
            return self.frame_count, self.read_errors

    def stop(self) -> None:
        self.stop_event.set()


def decode_mjpeg(frame):
    """Decode one raw MJPEG packet only when a preview is requested."""
    if frame is None:
        return None
    return cv2.imdecode(frame.reshape(-1), cv2.IMREAD_COLOR)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="ICM-42688 + camera monitor using the camera's internal 60 Hz mode"
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--spi", default="/dev/spidev0.0")
    parser.add_argument("--speed", type=int, default=1_000_000)
    # The YLX-2UQ2 advertises 60 FPS only for 3840x1080, 1920x1080 and
    # 1920x1072 MJPG. 1280x480 is a separate 210 FPS transport mode.
    parser.add_argument("--width", type=int, default=3840)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=60.0)
    parser.add_argument("--output", default="camera2_internal_60hz")
    parser.add_argument("--no-preview", action="store_true")
    parser.add_argument(
        "--decoded-capture",
        action="store_true",
        help="let OpenCV decode every MJPEG frame (CPU intensive at 3840x1080)",
    )
    args = parser.parse_args()

    if args.fps <= 0:
        parser.error("--fps must be greater than zero")
    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    imu = None
    cap = None
    reader = None

    try:
        imu = ICM42688(args.spi, args.speed)
        imu.initialize_200_hz()
        print(f"[IMU] WHO_AM_I=0x{imu._read_u8(imu.WHO_AM_I):02X}, initialized")

        # Configure the kernel device explicitly. CAP_PROP_FPS alone may only
        # report OpenCV's requested value instead of the actual timeperframe.
        configure_v4l2_mode(args.device, args.width, args.height, args.fps)

        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open camera: {args.device}")

        raw_mjpeg = not args.decoded_capture
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        # Test version: use the OpenCV/V4L2 default capture-buffer count.
        # cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
        if raw_mjpeg:
            # Return the compressed MJPEG packet instead of decoding every
            # 3840x1080 frame to BGR. This makes frame-rate measurement reflect
            # USB/V4L2 delivery rather than the board's JPEG decoder speed.
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        # Set FPS last: some V4L2 backends renegotiate/reset the frame interval
        # when FOURCC, dimensions, buffer count or conversion mode changes.
        cap.set(cv2.CAP_PROP_FPS, args.fps)

        negotiated_fps = cap.get(cv2.CAP_PROP_FPS)
        print(
            f"[Camera] {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ "
            f"{negotiated_fps:.2f} FPS"
        )
        if abs(negotiated_fps - args.fps) > 0.5:
            print(
                f"[Camera] WARNING: V4L2 did not negotiate {args.fps:.2f} FPS. "
                "Select an exact mode reported by v4l2-ctl --list-formats-ext."
            )

        reader = CameraReader(cap, raw_mjpeg)
        reader.start()
        print("[Main] internal/free-running capture active; press q or Ctrl+C to stop")

        last_count = 0
        last_time = time.monotonic()
        frame_id = 0

        while True:
            # In headless mode do not copy a large MJPEG packet every 10 ms.
            # Fetch a frame only when the once-per-second save is due.
            frame = None if args.no_preview else reader.get_latest_frame()
            display_frame = None
            if frame is not None and not args.no_preview:
                display_frame = decode_mjpeg(frame) if raw_mjpeg else frame
                if display_frame is not None:
                    cv2.imshow("Camera - Internal 60 Hz", display_frame)

            now = time.monotonic()
            if now - last_time >= 1.0:
                current_count, read_errors = reader.get_counters()
                measured_fps = (current_count - last_count) / (now - last_time)

                try:
                    temperature, accel, gyro, _ = imu.read_sample()
                    imu_text = (
                        f"T={temperature:6.2f} C "
                        f"A=({accel[0]:+7.3f},{accel[1]:+7.3f},{accel[2]:+7.3f}) g "
                        f"G=({gyro[0]:+8.2f},{gyro[1]:+8.2f},{gyro[2]:+8.2f}) dps"
                    )
                except Exception as exc:
                    imu_text = f"read failed: {exc}"

                print(
                    f"[IMU] {imu_text} | [Camera] frames={current_count}, "
                    f"measured={measured_fps:.1f} FPS, errors={read_errors}"
                )
                if current_count > 0 and measured_fps < args.fps * 0.90:
                    print(
                        f"[Camera] WARNING: receive rate is below the requested "
                        f"{args.fps:.1f} FPS; check USB mode/bandwidth and frame loss"
                    )

                if current_count != last_count:
                    if frame is None:
                        frame = reader.get_latest_frame()
                    filename = output_dir / f"image_{frame_id:06d}.jpg"
                    if frame is None:
                        print("[Camera] warning: no frame available to save")
                    elif raw_mjpeg:
                        filename.write_bytes(frame.tobytes())
                    elif not cv2.imwrite(str(filename), frame):
                        print(f"[Camera] warning: failed to save {filename}")
                    frame_id += 1

                last_count = current_count
                last_time = now

            if args.no_preview:
                time.sleep(0.01)
            elif cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\n[Main] interrupted by Ctrl+C")
    finally:
        if reader is not None:
            reader.stop()
        if cap is not None:
            cap.release()
        if reader is not None:
            reader.join(timeout=1.0)
        if imu is not None:
            imu.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
