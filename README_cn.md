```markdown
# Road Crack Inspection Robot
## Project Overview
Based on RDK X5 board and USB depth camera, this project builds a full perception-decision-actuation robot closed-loop system, realizing automatic pavement defect inspection based on ROS2 and YOLO.

The robot keeps moving straight during patrol; the camera captures real-time road images, and BPU-accelerated YOLO detects road defects with multi-frame verification to filter false detection. When defects are detected, the robot automatically stops for 3 seconds, uploads defect images and detection results to Django backend via Wi-Fi. MinIO stores pictures and MySQL stores structured data. The robot resumes straight patrol after upload completes. This lightweight deployment is suitable for automatic inspection of village lanes and factory roads.

RDK X5 + USB depth camera autonomous pavement inspection robot with full perception-decision-actuation loop based on ROS2 & YOLO. The robot cruises straight continuously; BPU-accelerated YOLO detects road defects with multi-frame validation to avoid false alarms. When defects are detected, the vehicle stops for 3s, uploads images & detection data to Django backend via Wi-Fi. MySQL stores structured records and MinIO stores defect images. Lightweight deployment fits village & factory road inspection.

## Hardware List
1. RDK X5 Development Board
2. USB Depth Camera
3. Two-wheel differential drive inspection chassis (standard chassis for Digua Track)

## Software Environment
### Robot Terminal (RDK X5)
- System: Ubuntu 22.04 aarch64
- Framework: ROS 2 Humble / TogetheROS.Bot
- Algorithm: YOLO pavement defect detection (BPU hardware acceleration)
### Cloud Supporting Service
- Web: Django web backend
- Database: MySQL
- Object Storage: MinIO

## Core Self-developed ROS Code Structure
road_inspect_ws/src └── road_defect_detect
# Full-link integrated functions:
# 1. Perception: USB depth camera image capture
# 2. Decision: YOLO defect inference, multi-frame anti-false detection, state machine scheduling
# 3. Actuation: Chassis straight movement, stop for 3 seconds when detecting defects
# 4. Communication: Upload images & detection results to Django via WiFi HTTP interface
# 5. Startup: Built-in one-click multi-node launch logic

## Quick Start (Robot Side)
1. Put the package into the src folder of ROS2 workspace
2. Build source code
```bash
colcon build
3. Load environment variables
source install/setup.bash
4. Launch full automatic inspection system
ros2 launch road_defect_detect main.launch.py

Running Workflow
Robot straight patrol → Real-time defect identification & verification → Stop for 3s after detecting defects → Upload images & detection data to cloud → Continue straight patrol
