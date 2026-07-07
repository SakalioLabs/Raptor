# 基于 RK3566 的多旋翼无人机飞控实现方案

## 1. RK3566 SoC 概述

### 1.1 核心规格

Rockchip RK3566 是一款面向 AIoT 场景的高性价比 SoC，具备以下核心特性：

| 参数 | 规格 |
|------|------|
| CPU | 四核 Cortex-A55 @ 1.8GHz |
| GPU | Mali-G52 2EE, 支持 OpenGL ES 3.2 / Vulkan 1.1 |
| NPU | 0.8 TOPS (INT8), 支持 INT8/INT16/FP16 混合运算 |
| 内存 | 支持 LPDDR4/LPDDR4X, 最大 8GB |
| 视频解码 | 4K@60fps H.265/H.264/VP9 |
| 视频编码 | 1080p@60fps H.265/H.264 |
| 显示 | 支持 MIPI DSI / eDP / RGB / LVDS |
| 接口 | 3x SPI, 6x I2C, 10x UART, 2x USB 2.0, USB 3.0 OTG, PCIe 2.1, SDIO 3.0 |
| 存储 | eMMC 5.1, SDIO, SPI NAND/NOR |

### 1.2 飞控场景适配性分析

RK3566 在多旋翼飞控体系中的定位是 **伴随计算机（Companion Computer）**，而非直接替代传统 MCU 级飞控（如 STM32F7/H7）。这一定位基于以下考量：

- **实时性**：Cortex-A55 运行 Linux 通用操作系统，无法保证微秒级中断响应；姿态环控制仍需专用 MCU 处理。
- **算力优势**：0.8 TOPS NPU + 四核 CPU 可同时处理视觉感知、AI 推理、通信协议栈等高算力任务。
- **丰富外设**：MIPI CSI 可直接接入高分辨率相机，PCIe/USB 3.0 可扩展 4G/5G 模组。

## 2. 硬件系统架构

### 2.1 双处理器架构

```
┌─────────────────────────────────────────────────────────┐
│                    飞控核心板                              │
│                                                          │
│  ┌──────────────────┐     UART/SPI      ┌─────────────┐│
│  │    RK3566         │◄────────────────►│  STM32H743   ││
│  │  (伴随计算机)      │    MAVLink       │ (实时飞控MCU)  ││
│  │                    │                  │               ││
│  │  - AI 推理         │                  │ - 姿态控制    ││
│  │  - 视觉处理         │                  │ - 传感器融合  ││
│  │  - 通信管理         │                  │ - 电调驱动    ││
│  │  - 任务规划         │                  │ - 安全监控    ││
│  └──────┬───────────┘                  └──────┬───────┘│
│         │                                      │        │
│    ┌────┴────┐                          ┌──────┴──────┐ │
│    │ MIPI CSI│                          │  IMU/GPS    │ │
│    │ 摄像头   │                          │  气压计/磁力计│ │
│    └─────────┘                          └─────────────┘ │
└─────────────────────────────────────────────────────────┘
         │                                      │
    ┌────┴────────┐                     ┌───────┴──────┐
    │ 4G/5G 模组  │                     │ 电调 × 6      │
    │ (RM500U等)  │                     │ (BLHeli_32)   │
    └─────────────┘                     └──────────────┘
```

### 2.2 关键外设连接

| 外设 | 接口 | 用途 |
|------|------|------|
| STM32H743 飞控 | UART (1.5Mbps) | MAVLink 通信 |
| MIPI CSI 摄像头 | MIPI CSI-2 (2-lane) | 视觉感知 |
| 4G/5G 模组 | USB 3.0 | 蜂窝网络通信 |
| GPS/BDS 模组 | UART | 高精度定位 |
| TF 卡 | SDIO 3.0 | 数据记录与模型存储 |
| PWM 电调 | GPIO (via STM32) | 电机控制 |
| 激光雷达 | USB 2.0 / UART | 避障与建图 |

### 2.3 电源设计

```
电池 (6S LiPo 22.2V)
    │
    ├── BMS 保护板
    │       │
    │       ├── 降压至 5V (3A) → RK3566 核心板
    │       ├── 降压至 5V (2A) → STM32 飞控 + 传感器
    │       └── 降压至 3.3V → GPS / 通信模组
    │
    └── 电调 → 电机
```

## 3. 软件架构

### 3.1 分层架构

```
┌─────────────────────────────────────────────┐
│              应用层 (Application)             │
│  任务规划 │ 自主避障 │ 目标追踪 │ 巡检/物流   │
├─────────────────────────────────────────────┤
│              AI 推理层 (RKNN Runtime)         │
│  目标检测 │ 语义分割 │ 姿态估计 │ 异常检测    │
├─────────────────────────────────────────────┤
│              中间件层 (Middleware)             │
│  MAVSDK │ ROS2 │ DDS │ GStreamer            │
├─────────────────────────────────────────────┤
│              系统层 (OS)                      │
│  Linux Kernel 5.10 (RT-PREEMPT 补丁)         │
├─────────────────────────────────────────────┤
│              驱动层 (Drivers)                 │
│  V4L2 │ DRM │ RKNN Driver │ 网络驱动        │
├─────────────────────────────────────────────┤
│              硬件抽象层 (HAL)                 │
│  Rockchip BSP │ U-Boot                       │
└─────────────────────────────────────────────┘
```

### 3.2 实时操作系统选型

| 方案 | 优势 | 劣势 | 适用场景 |
|------|------|------|---------|
| Linux + RT-PREEMPT | 生态丰富，驱动完善 | 延迟约 50-100μs | 伴随计算机 |
| Linux + Xenomai | 硬实时，延迟 < 10μs | 配置复杂 | 需要准实时的场景 |
| Zephyr (on STM32) | 硬实时，轻量 | 需额外 MCU | 姿态环控制 |

**推荐方案**：RK3566 运行 Linux (Buildroot/Yocto) + RT-PREEMPT 补丁，STM32H743 运行 PX4 NuttX，通过 MAVLink 解耦实时与非实时任务。

### 3.3 飞控软件栈

#### PX4 适配方案

```
RK3566 (Linux)                      STM32H743 (NuttX)
┌─────────────────┐                 ┌─────────────────┐
│  MAVSDK (C++)   │                 │  PX4 固件        │
│  ┌───────────┐  │   MAVLink       │                 │
│  │ 任务管理器 │  │◄──(UART)──────►│  姿态控制器      │
│  │ 航线规划   │  │                 │  位置控制器      │
│  │ 避障逻辑   │  │                 │  传感器驱动      │
│  └───────────┘  │                 │  电调驱动        │
│  ┌───────────┐  │                 └─────────────────┘
│  │ RKNN 推理  │  │
│  │ 视觉处理   │  │
│  └───────────┘  │
│  ┌───────────┐  │
│  │ 4G/5G 通信 │  │
│  │ 视频推流   │  │
│  └───────────┘  │
└─────────────────┘
```

## 4. 控制算法实现

### 4.1 姿态控制（运行在 STM32）

```c
// PID 姿态控制器伪代码
typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
} PID_Controller;

float pid_update(PID_Controller* pid, float error, float dt) {
    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}

// 姿态环 (1kHz)
void attitude_control(Quaternion current, Quaternion desired) {
    Quaternion error = quaternion_multiply(desired, quaternion_inverse(current));
    Euler euler_error = quaternion_to_euler(error);
    
    float roll_cmd  = pid_update(&roll_pid,  euler_error.roll,  0.001f);
    float pitch_cmd = pid_update(&pitch_pid, euler_error.pitch, 0.001f);
    float yaw_cmd   = pid_update(&yaw_pid,   euler_error.yaw,   0.001f);
    
    // 混控输出到各电机
    motor_mixer(roll_cmd, pitch_cmd, yaw_cmd, throttle);
}
```

### 4.2 视觉避障（运行在 RK3566）

```python
# RKNN 目标检测推理示例
from rknnlite.api import RKNNLite

rknn = RKNNLite()
rknn.load_rknn('yolov5s.rknn')
rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0)

def obstacle_detection(frame):
    # 预处理
    img = preprocess(frame, size=(640, 640))
    # NPU 推理 (~30ms @ 640x640)
    outputs = rknn.inference(inputs=[img])
    # 后处理 NMS
    boxes, scores, classes = postprocess(outputs, conf_threshold=0.5)
    
    # 计算避障向量
    avoidance_vector = compute_avoidance(boxes, scores, depth_map)
    return avoidance_vector
```

### 4.3 导航与路径规划

```python
# A* 路径规划 + 动态避障
class Navigator:
    def __init__(self):
        self.occupancy_grid = OccupancyGrid(resolution=0.5)  # 0.5m 分辨率
        self.path_planner = AStarPlanner()
    
    def update_map(self, point_cloud, detection_results):
        """融合激光雷达点云和视觉检测结果更新占据栅格"""
        self.occupancy_grid.update(point_cloud)
        for det in detection_results:
            self.occupancy_grid.mark_obstacle(det.position, det.size)
    
    def plan_path(self, start, goal):
        """规划全局路径并做局部避障修正"""
        global_path = self.path_planner.plan(self.occupancy_grid, start, goal)
        local_path = self.dynamic_avoidance(global_path)
        return local_path
    
    def dynamic_avoidance(self, global_path):
        """基于 VFH+ 算法做局部避障"""
        # 优先使用全局路径，遇到动态障碍时局部修正
        return vfh_plus(global_path, self.occupancy_grid)
```

## 5. 性能指标与约束

### 5.1 计算资源分配

| 任务 | 处理器 | CPU 占用 | 延迟要求 |
|------|--------|---------|---------|
| 姿态控制 | STM32H743 | N/A (裸机/NuttX) | < 1ms |
| 目标检测 | RK3566 NPU | ~30% NPU | < 50ms |
| 视频编码 | RK3566 VPU | ~10% CPU | < 33ms (30fps) |
| 路径规划 | RK3566 CPU | ~20% CPU | < 100ms |
| 4G 通信 | RK3566 CPU | ~5% CPU | < 200ms |
| 数据记录 | RK3566 CPU | ~5% CPU | 异步 |

### 5.2 功耗预算

| 模块 | 典型功耗 |
|------|---------|
| RK3566 核心板 | 2-3W |
| STM32H743 + 传感器 | 0.5W |
| 4G 模组 | 1-2W |
| 摄像头 | 0.5W |
| GPS | 0.2W |
| **总计** | **4-6W** |

## 6. 开发环境搭建

### 6.1 编译工具链

```bash
# 交叉编译工具链
sudo apt install gcc-aarch64-linux-gnu g++-aarch64-linux-gnu

# Buildroot 构建系统
git clone https://github.com/buildroot/buildroot.git
cd buildroot
make rockchip_rk3566_defconfig
make -j$(nproc)

# RKNN SDK
git clone https://github.com/airockchip/rknn-toolkit2.git
pip install rknn-toolkit2/packages/rknn_toolkit2-*-cp310-cp310-linux_x86_64.whl
```

### 6.2 PX4 固件编译

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
make px4_fmu-v6c_default   # 适配 STM32H743
```

## 7. 风险与挑战

| 风险 | 影响 | 缓解策略 |
|------|------|---------|
| Linux 内核实时性不足 | 姿态环抖动 | 姿态控制下沉到 STM32 |
| NPU 算力有限 (0.8 TOPS) | 复杂模型推理慢 | 使用量化模型 + 模型剪枝 |
| 温度范围限制 | 高温降频 | 散热设计 + 动态调频 |
| 4G 延迟波动 | 遥控响应延迟 | 本地自主决策 + 断联保护 |
| EMI 干扰 | 传感器数据异常 | PCB 布局优化 + 屏蔽设计 |
