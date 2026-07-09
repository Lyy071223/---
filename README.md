道路裂缝检测机器人
## Project Overview
本项目基于RDK X5开发板+USB深度相机，搭建完整**感知-决策-执行**机器人闭环，基于ROS2、YOLO实现路面病害自主巡检。
小车默认持续直行巡检；相机实时采集画面，BPU加速YOLO检测病害，搭配多帧校验过滤误判。识别到病害后小车自动停车3秒，通过WiFi将病害图片、检测结果上传至Django后台，MinIO存储图像、MySQL存储结构化数据，上传完成后恢复直行巡检。整体轻量化部署，适配乡村小路、厂区道路自动化巡检。

RDK X5 + USB depth camera autonomous pavement inspection robot with full perception-decision-actuation loop based on ROS2 & YOLO. The robot cruises straight continuously; BPU-accelerated YOLO detects road defects with multi-frame validation to avoid false alarms. When defects are detected, the vehicle stops for 3s, uploads images & detection data to Django backend via Wi-Fi. MySQL stores structured records and MinIO stores defect images. Lightweight deployment fits village & factory road inspection.

## Hardware List
1. RDK X5 开发板
2. USB 深度相机
3. 两轮差速巡检小车底盘（地瓜赛道标准底盘）

## Software Environment
### 小车端（RDK X5）
- System: Ubuntu 22.04 aarch64
- Framework: ROS 2 Humble / TogetheROS.Bot
- Algorithm: YOLO 路面病害检测（BPU硬件加速）
### 云端配套服务
- Web: Django 网页后台
- Database: MySQL
- Object Storage: MinIO

## Core Self-developed ROS Code Structure
road_inspect_ws/src
└── road_detect_node    
    # 集成全链路逻辑：
    # 1.感知：USB深度相机图像采集
    # 2.决策：YOLO病害推理、多帧校验防误判、状态机调度
    # 3.执行：小车底盘直行、病害定点停车3s控制
    # 4.通信：WiFi HTTP接口上传图像与检测结果
    # 5.启动：内置多节点一键启动逻辑


## Quick Start (Robot Side)
1. 将功能包放入ROS2工作空间src目录
2. 编译代码
```bash
colcon build
3.加载环境
source install/setup.bash
4.启动自主巡检全流程
ros2 launch inspect_launch main.launch.py

运行流程
小车直行巡检 → 实时病害识别校验 → 检出病害停车 3s → 上传图像与检测数据到云端 → 继续直行
