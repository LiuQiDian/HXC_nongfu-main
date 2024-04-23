# 重庆邮电大学HXC战队的农夫机器人代码仓库
## 使用esp32 S3主控 platform io arduino框架开发
### 主函数为/src/main.cpp
### 其他模块在/include
## 模块介绍
### C600.hpp
  用于驱动大疆电机的库，CAN通信相关也在此文件
### car.hpp
  用于驱动麦克纳姆轮底盘的库
### HEServo.hpp
  用于驱动总线舵机的库
### PID_CONTROL.hpp
  本仓库的控制所使用的PID控制器，可以实现时间上平滑的积分和微分控制，死区控制，输出限幅
