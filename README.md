# ICM-42688 与双目camera测试程序

本项目用于在 Linux 环境下验证 ICM-42688 IMU 的 SPI 通信，并监测摄像头在内部运行模式或外部触发模式下的实际帧率。监控程序会同时输出 IMU 温度、三轴加速度、三轴角速度，以及摄像头累计帧数、实测帧率和读取错误数。

## 1. 环境准备

### 硬件与设备节点

- ICM-42688 默认使用 SPI 设备 `/dev/spidev0.0`。
- V4L2 摄像头默认使用视频设备 `/dev/video0`。
- 外部触发模式需要接入 **1.8 V、60 Hz** 的外部触发信号。

测试前可确认设备节点是否存在：

```bash
ls -l /dev/spidev0.0 /dev/video0
```

### 软件依赖

程序依赖 Python 3、OpenCV、python-periphery 和 `v4l2-ctl`：

```bash
sudo apt update
sudo apt install -y python3 python3-opencv python3-pip v4l-utils
python3 -m pip install python-periphery
```

如果系统禁止向全局 Python 环境安装软件包，请在虚拟环境中安装 `python-periphery`，或使用系统发行版提供的软件包。

## 2. 验证 IMU 的 SPI 通信

进入项目目录后执行：

```bash
sudo python3 ./icm42688_test.py \
  --spi /dev/spidev0.0 \
  --speed 100000 \
  id
```

SPI 通信正常时，预期输出为：

```text
WHO_AM_I=0x47
```

`0x47` 是 ICM-42688 的器件标识。如果返回其他值或出现 SPI 读写错误，请再尝试几次，或许存在接触不良。

## 3. 相机内部运行模式

执行：

```bash
sudo python3 ./monitor_test_internal.py
```

程序默认执行以下操作：

1. 通过 `/dev/spidev0.0` 初始化 ICM-42688，并校验 `WHO_AM_I=0x47`。
2. 将摄像头配置为 MJPEG、`3840 × 1080 @ 60 FPS`，使用摄像头内部时钟连续拉流。
3. 每秒打印一次 IMU 温度、三轴加速度、三轴角速度，以及摄像头累计帧数、实测帧率和读取错误数。
4. 大约每秒保存一帧图像。默认保存目录为 `camera2_internal_60hz/`，文件名格式为 `image_000000.jpg`。

示例输出：

```text
[IMU] WHO_AM_I=0x47, initialized
[Camera] 3840x1080 @ 60.00 FPS
[IMU] T= 25.00 C A=(...) g G=(...) dps | [Camera] frames=60, measured=60.0 FPS, errors=0
```

完整参数示例：

```bash
sudo python3 ./monitor_test_internal.py \
  --device /dev/video0 \
  --spi /dev/spidev0.0 \
  --speed 1000000 \
  --width 3840 \
  --height 1080 \
  --fps 60 \
  --output camera2_internal_60hz \
```

常用选项：

- `--output`：指定抓帧保存目录。
- `--no-preview`：可关闭预览，适用于无桌面或远程终端环境。

## 4. 相机外部触发模式

执行：

```bash
sudo python3 ./monitor_test_trigger.py
```

程序执行流程如下：

1. 通过 `/dev/spidev0.0` 初始化 ICM-42688，并校验 `WHO_AM_I=0x47`。
2. 使用 OpenCV 的 V4L2 后端打开摄像头并开始拉流。
3. 拉流启动后等待 0.5 秒，再自动调用 `v4l2-ctl` 设置 `backlight_compensation=1`。
4. 接入 **1.8 V、60 Hz** 外部触发信号。
5. 每秒打印 IMU 数据、摄像头累计帧数、实测帧率和读取失败次数。外部触发稳定后，实测帧率应趋近 `60.0 FPS`。

> **重要：** 先启动摄像头拉流，再设置 `backlight_compensation=1`（BLS=1），否则外部触发无法启动。

外部触发脚本默认请求 MJPEG、`1280 × 480 @ 210 FPS` 的摄像头传输模式。这里的 `210 FPS` 是请求配置；接入 60 Hz 外部触发信号后，程序打印的**实测帧率**应趋近 `60.0 FPS`。

示例输出：

```text
[IMU] WHO_AM_I=0x47, initialized
[Camera] 实际参数：1280x480 @ 210.00 FPS
[V4L2] backlight_compensation: 1
[IMU] T= 25.00 °C A=(...) g G=(...) dps | [Camera] 帧数=60, 测量帧率=60.0 FPS, 读取失败=0
```

完整参数示例：

```bash
sudo python3 ./monitor_test_trigger.py \
  --device /dev/video0 \
  --spi /dev/spidev0.0 \
  --speed 1000000 \
  --width 1280 \
  --height 480 \
  --fps 210 \
  --delay 0.5 \
```

常用选项：

- `--delay`：开始拉流后，等待多少秒再设置 BLS=1。
- `--no-preview`：关闭预览窗口，适用于无桌面环境。

外部触发程序会尝试将图像保存到 `camera2/`。当前脚本不会自动创建该目录，因此运行前应执行：

```bash
mkdir -p camera2
```

## 5. 退出程序

在终端中按 `Ctrl+C` 可安全停止程序并释放摄像头和 SPI 设备。启用预览窗口时，也可以在预览窗口中按 `q` 退出。

## 6. 常见问题

### `WHO_AM_I` 不是 `0x47`

信号或许不稳定，请再尝试几次

### 无法打开摄像头

检查摄像头设备节点和支持的采集格式：

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

如果摄像头不是 `/dev/video0`，请使用 `--device` 指定正确节点。


### 无图形桌面或 OpenCV 窗口报错

运行脚本时添加 `--no-preview`。
