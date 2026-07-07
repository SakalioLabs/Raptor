# Raptor - 下一代空中智能平台 (前沿研究)

基于 **RK3566 SoC** 的多旋翼无人机飞控系统，集成 NPU 边缘智能与 4G-6G 蜂窝通信。
本项目持续跟踪前沿论文并实现可验证的算法模块。

## 测试状态

**80/80 单元测试通过** (pytest, ~10s)

## 项目结构

```
Raptor/
├── docs/                          # 研究文档
│   ├── 01-rk3566-flight-controller.md
│   ├── 02-npu-applications-in-uav.md
│   ├── 03-4g-6g-air-intelligence-platform.md
│   └── 04-system-architecture.md
├── src/raptor/                    # 核心实现 (20 modules)
│   ├── flight/                    # 飞控核心
│   │   ├── pid_controller.py      # PID控制器 (6 tests)
│   │   ├── attitude.py            # 姿态估计/四元数 (6 tests)
│   │   ├── motor_mixer.py         # 六旋翼混控器 (3 tests)
│   │   └── state_machine.py       # 飞行状态机 (5 tests)
│   ├── control/                   # 前沿控制
│   │   └── mpc_controller.py      # MPC模型预测控制 (4 tests)
│   ├── vision/                    # 视觉感知
│   │   ├── obstacle_detector.py   # 障碍物检测/避障 (3 tests)
│   │   ├── landing_detector.py    # 精准降落检测 (3 tests)
│   │   └── depth_estimator.py     # 单目深度估计 (2 tests)
│   ├── ai/                        # 边缘AI
│   │   └── edge_detector.py       # AnchorFree轻量检测 (5 tests)
│   ├── slam/                      # 自主导航
│   │   └── visual_inertial_odometry.py  # VIO惯性里程计 (4 tests)
│   ├── comm/                      # 通信层
│   │   ├── mavlink_handler.py     # MAVLink协议 (5 tests)
│   │   ├── link_manager.py        # 多链路管理 (3 tests)
│   │   └── network_monitor.py     # 网络质量监控 (4 tests)
│   ├── mission/                   # 任务规划
│   │   ├── waypoint.py            # 航点导航 (4 tests)
│   │   ├── path_planner.py        # A*路径规划 (3 tests)
│   │   └── geofence.py            # 地理围栏 (3 tests)
│   ├── planning/                  # 前沿规划
│   │   └── cooperative_planner.py # CBS多智能体协同 (3 tests)
│   ├── platform/                  # 平台能力
│   │   ├── task_offloader.py      # 边云任务卸载 (4 tests)
│   │   └── swarm_coordinator.py   # 集群协调 (6 tests)
│   └── optimization/              # 轨迹优化
│       └── energy_optimizer.py    # 能量感知轨迹优化 (4 tests)
└── tests/                         # 单元测试 (80个)
```

## 前沿论文参考

| 模块 | 参考论文 | 年份 |
|------|---------|------|
| MPC Controller | Camacho & Bordons, "Model Predictive Control" | 2007 |
| MPC Controller | Richter et al., "Real-Time Trajectory Generation" | 2013 |
| VIO | Mourikis & Roumeliotis, "Multi-State Constraint KF" | 2007 |
| Edge Detector | Wang et al., "YOLOv7: Trainable bag-of-freebies" | 2022 |
| Edge Detector | MobileNetV3 architecture principles | 2019 |
| CBS Planner | Sharon et al., "Conflict-based search for MAPF" | 2015 |
| Energy Optimizer | Richter et al., "Polynomial Trajectory Planning" | 2013 |
| Energy Optimizer | Lin et al., "Energy-aware trajectory planning" | 2020 |

## 快速开始

```bash
pip install -r requirements.txt
python -m pytest tests/ -v
```

## 许可证

MIT License
