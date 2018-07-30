## 简介

`ros_ilins` 这个包是为 Inertial Labs 的 INS-D 编写的驱动程序, 目前仅针对 INS-D COM1 接口输出的NMEA和OPVT2A协议做了解析

## ROS包结构

- ros_ilins
	- ilins: 用于组织子包, 无实际功能
	- ilins_driver: 与串口或文件系统通信, 解析数据, 并发布至ros节点
	- ilins_msgs: 通过ros的message系统定义待解析的协议
    - ilins_utils: 订阅节点例程, 可修改后用于转发消息

### 启动

- `roslaunch ilins_driver il_ins.launch`  启动驱动程序
- `rosrun ilins_utils ilins_subscriber_demo`  启动订阅例程

### 结束

- `rosnode kill ilins_node` (推荐使用)
- 发送 SIGINT 信号到 ilins_node 进程, 如 `ps -aux | grep ilins_node | awk '{print $2}' | xargs kill -INT`

### launch 文件

- deviceName: 用于标识设备名称
- protocol: 指定待解析协议, 目前支持 NMEA(PAPR) 和 OPVT2A 两种协议
- mode: 文件回放设置, 用于指定文件回放模式, 循环模式 或 单次模式
- record: 是否启用文件录制, 该设置在文件回放模式下不可用
- serial_port: 指定串口设备路径
- baudrate: 指定串口波特率
- replay_file: 指定用于回放的数据文件, 如果使用串口, 请注释该条内容

## 完成的工作

- NMEA(PAPR) & OPVT2A 协议解析
- 文件回放
- 数据包录制, 单模块调试用
- 节点诊断信息
- 订阅例程

## TODO

- Autoware 适配