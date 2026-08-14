sudo python3 ./icm42688_test.py \
  --spi /dev/spidev0.0 \
  --speed 100000 \
  id
  
运行该程序验证SPI正常通信，正常输出：WHO_AM_I=0X47

（1）相机内部运行模式
sudo python3 ./monitor_test_internal.py

- 运行之后先通过SPI验证：0X47
- 输出 ICM-42688（IMU）的六轴数据与摄像头内部帧率：3840✖1080 @ 60fps，并自动保存抓取的帧
- 同时打印温度/加速度/陀螺数据+帧率/保存图像总数frame


（2）相机外接触发模式
sudo python3 ./ monitor_test_trigger.py

- 运行之后同样先通过SPI验证：0X47，脚本初始化并读取IMU温度/加速度/陀螺数据
- 使用 OpenCV 从 V4L2 摄像头拉流
- 在启动拉流后程序自动调用 `v4l2-ctl` 设置 `backlight_compensation=1`
（必须先拉流，再设置BLS=1，否则外部触发无法启动）

- 此时接入1.8V外部触发信号，读取帧数会转为60.0
- 测量并打印IMU数据+相机实际帧率与读取错误数

  输入“ctrl+c“可退出运行
