python monitor_test_internal.py

同时输出 ICM-42688（IMU）的六轴数据与摄像头内部帧率：3080✖1080 @ 60fps，并可保存抓取的帧
- 初始化并读取 ICM-42688（通过 SPI：WHO_AM_I=0x47），打印温度/加速度/陀螺数据
- 测量相机内部帧率，打印相关信息



python monitor_test_trigger.py

同时输出 ICM-42688（IMU）的六轴数据与摄像头外部触发测得的帧率，并可保存抓取的帧。

- 初始化并读取 ICM-42688（通过 SPI：WHO_AM_I=0x47），打印温度/加速度/陀螺数据。
- 使用 OpenCV 从 V4L2 摄像头拉流，测量并打印实际帧率与读取错误数。
- 在启动拉流后调用 `v4l2-ctl` 设置 `backlight_compensation=1`
- 接入1.8V外部触发信号，打印相关信息


