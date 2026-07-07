# Raptor - 下一代空中智能平台

基于 **RK3566 SoC** 的多旋翼无人机飞控系统，集成 NPU 边缘智能与 4G-6G 蜂窝通信，面向下一代空中智能平台。

## 测试状态

**60/60 单元测试通过** (pytest, 1.16s)

## 项目结构

```
Raptor/
├── docs/                          # 研究文档
│   ├── 01-rk3566-flight-controller.md
│   ├── 02-npu-applications-in-uav.md
│   ├── 03-4g-6g-air-intelligence-platform.md
│   └── 04-system-architecture.md
├── src/raptor/                    # 核心实现
│   ├── flight/                    # 飞控核心
│   │   ├── pid_controller.py      # PID控制器 (6 tests)
│   │   ├── attitude.py            # 姿态估计/四元数 (6 tests)
│   │   ├── motor_mixer.py         # 六旋翼混控器 (3 tests)
│   │   └── state_machine.py       # 飞行状态机 (5 tests)
│   ├── vision/                    # 视觉感知
│   │   ├── obstacle_detector.py   # 障碍物检测/避障 (3 tests)
│   │   ├── landing_detector.py    # 精准降落检测 (3 tests)
│   │   └── depth_estimator.py     # 单目深度估计 (2 tests)
│   ├── comm/                      # 通信层
│   │   ├── mavlink_handler.py     # MAVLink协议处理 (5 tests)
│   │   ├── link_manager.py        # 多链路管理 (3 tests)
│   │   └── network_monitor.py     # 网络质量监控 (4 tests)
│   ├── mission/                   # 任务规划
│   │   ├── waypoint.py            # 航点导航 (4 tests)
│   │   ├── path_planner.py        # A*路径规划 (3 tests)
│   │   └── geofence.py            # 地理围栏 (3 tests)
│   └── platform/                  # 平台能力
│       ├── task_offloader.py      # 边云任务卸载 (4 tests)
│       └── swarm_coordinator.py   # 集群协调 (6 tests)
├── tests/                         # 单元测试 (60个)
├── requirements.txt
└── pytest.ini
```

## 快速开始

```bash
pip install -r requirements.txt
python -m pytest tests/ -v
```

## 技术架构

| 层级 | 模块 | 技术 |
|------|------|------|
| 飞控 | PID + 姿态 + 混控 + 状态机 | STM32H743 NuttX |
| 视觉 | 目标检测 + 深度估计 + 避障 | RK3566 NPU (RKNN) |
| 通信 | MAVLink + 多链路 + 网络监控 | 4G/5G LTE |
| 规划 | 航点 + A* + 地理围栏 | Linux RT-PREEMPT |
| 平台 | 任务卸载 + 集群协调 | MEC 边缘计算 |

## 许可证

MIT License
