# Raptor - 下一代空中智能平台研究报告

## 项目概述

本项目研究基于 **RK3566 SoC** 的多旋翼无人机飞控系统实现方案，探索 **NPU（神经网络处理单元）** 在无人机领域的应用潜力，并设计面向 **4G-6G 网络制式** 的下一代空中智能平台架构。

## 研究内容

### 1. [RK3566 多旋翼飞控实现方案](docs/01-rk3566-flight-controller.md)
- RK3566 SoC 架构分析与飞控适配
- 硬件系统设计（传感器、电调、通信模块）
- 软件架构（实时内核、PX4/ArduPilot 适配）
- 控制算法实现（姿态、导航、避障）

### 2. [NPU 在无人机中的应用潜力](docs/02-npu-applications-in-uav.md)
- RK3566 NPU 技术规格与能力分析
- 典型 AI 推理任务（目标检测、视觉SLAM、避障）
- 模型优化与部署（RKNN-Toolkit）
- 应用场景：农业、巡检、物流、安防

### 3. [4G-6G 空中智能平台架构](docs/03-4g-6g-air-intelligence-platform.md)
- 4G/5G/6G 通信技术在无人机中的应用
- 网络切片与 QoS 保障
- 边缘计算与云端协同
- 超视距（BVLOS）通信架构

### 4. [系统架构总览与技术路线图](docs/04-system-architecture.md)
- 整体系统架构设计
- 硬件/软件技术栈
- 开发路线图与里程碑
- 风险分析与应对策略

## 技术栈

| 层级 | 技术选型 |
|------|---------|
| SoC | Rockchip RK3566 |
| NPU | RKNN 0.8 TOPS INT8 |
| 飞控 | PX4 / ArduPilot |
| OS | Linux (Buildroot/Yocto) |
| AI 框架 | RKNN-Toolkit, ONNX, TensorFlow Lite |
| 通信 | 4G LTE / 5G NR / 6G (预研) |
| 视觉 | OpenCV, GStreamer, MIPI CSI |

## 快速开始

```bash
# 克隆仓库
git clone https://github.com/SakalioLabs/Raptor.git
cd Raptor

# 查看研究文档
cat docs/01-rk3566-flight-controller.md
```

## 许可证

MIT License

## 联系方式

- 项目维护：SakalioLabs
- 邮箱：sakalioling@rankchord.com
