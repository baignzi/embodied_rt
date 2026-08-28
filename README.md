# EmbodiedRT — VLA实时推理与机械臂控制系统

> **一句话定位**：端到端 VLA（Vision-Language-Action）模型驱动的 7-DOF 机械臂实时控制系统，支持从自然语言指令 → 视觉推理 → 运动规划 → 实时控制的完整闭环。

---

## 1. 系统架构

```
┌──────────────┐   /vla/action_cmd    ┌──────────────────┐  /planning/trajectory  ┌──────────────────┐
│ VLA推理节点  │ ──────────────────▶  │  轨迹生成器      │ ─────────────────────▶ │ 实时控制器       │
│ (Python)     │   JSON[7-DOF动作]    │  (RRT*/线性插值) │   100Hz关节轨迹        │  (1000Hz PID)    │
│  mock/真实模型│                      └──────────────────┘                        └────────┬─────────┘
└──────┬───────┘                                                                         │ /control/joint_cmd
       │ /perception/observation                                                          ▼
       │  (相机图像)                                                            ┌──────────────────┐
       ▼                                                                        │ 安全监控         │
  [可选: 相机]                                                                  │ (限位/速度/力矩) │
                                                                                └──────────────────┘
```

**四个核心节点：**

| 节点 | 语言 | 功能 | 频率 |
|------|------|------|------|
| `vla_inference_node` | Python | VLA模型推理，输出7维动作指令 | 2~5 Hz |
| `trajectory_generator` | C++ | 将动作指令转化为关节空间轨迹 | 触发式 |
| `real_time_controller` | C++ | PID力矩控制，跟踪轨迹 | 1000 Hz |
| `safety_monitor` | C++ | 关节限位、速度、力矩监控 | 1000 Hz |

---

## 2. 两种运行模式

| 模式 | 说明 | 依赖 | 适用场景 |
|------|------|------|----------|
| **Mock 模式** ⭐推荐 | VLA输出模拟动作，轨迹用线性插值 | 只需 ROS2 | 演示、面试、无GPU环境 |
| **完整模式** | 真实VLA模型 + MoveIt2 RRT* | ROS2 + MoveIt2 + GPU(14GB显存) | 真实机器人、仿真实验 |

---

## 3. 快速开始（Ubuntu + Mock模式）

### 3.1 环境要求
- Ubuntu 22.04
- ROS2 Humble（推荐）或 Iron
- 至少 2GB 内存

### 3.2 安装 ROS2 Humble

```bash
# 1. 设置locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加ROS2源
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装ROS2 Humble（基础版即可）
sudo apt update
sudo apt install -y ros-humble-ros-base
source /opt/ros/humble/setup.bash
```

### 3.3 编译项目

```bash
# 创建工作空间
mkdir -p ~/embodied_rt_ws/src
cd ~/embodied_rt_ws/src

# 复制项目（或 git clone）
cp -r /path/to/embodied_rt .
cd ..

# 安装依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# source 工作空间
source install/setup.bash
```

### 3.4 一键运行（Mock模式）

```bash
# 一条命令启动全部4个节点
ros2 launch embodied_rt embodied_rt.launch.py
```

**预期输出：**
```
[vla_inference_node]: VLA Inference Node started in MOCK mode (2.0 Hz)
[trajectory_generator]: MoveIt2 NOT compiled in — using standalone linear interpolation fallback
[trajectory_generator]: Trajectory generated: 201 points, 2.00s total
[real_time_controller]: Real-time controller started at 1000 Hz
[safety_monitor]: Safety monitor active
```

### 3.5 查看运行状态

新开终端，查看各topic数据：

```bash
# 查看VLA动作输出
ros2 topic echo /vla/action_cmd --once

# 查看轨迹点数量
ros2 topic hz /planning/trajectory

# 查看控制指令
ros2 topic echo /control/joint_cmd --once

# 节点关系图
rqt_graph
```

---

## 4. 完整模式（VLA + MoveIt2）

### 4.1 安装 MoveIt2

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-moveit-resources-panda-description \
  ros-humble-robot-state-publisher
```

### 4.2 安装 VLA 依赖（需要GPU）

```bash
pip install torch transformers pillow numpy
# 模型 ~14GB，首次运行自动下载
# 需要 GPU 显存 >= 16GB（FP16）
```

### 4.3 重新编译 + 运行

```bash
cd ~/embodied_rt_ws
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash

# 完整模式：真实VLA模型 + RRT*规划
ros2 launch embodied_rt embodied_rt.launch.py
```

编译时会看到：
```
-- EmbodiedRT: MoveIt2 found — RRT* planner enabled
```

---

## 5. 单独运行各节点

### 5.1 VLA推理节点

```bash
# Mock模式（无需GPU）
ros2 run embodied_rt vla_inference_node --ros-args -p mock:=true -p rate_hz:=2.0

# 真实模型模式
ros2 run embodied_rt vla_inference_node

# 独立测试（不依赖ROS2）
python3 src/vla_inference_node.py --standalone --mock
```

### 5.2 轨迹生成器

```bash
ros2 run embodied_rt trajectory_generator
```

### 5.3 实时控制器

```bash
ros2 run embodied_rt real_time_controller
```

### 5.4 安全监控

```bash
ros2 run embodied_rt safety_monitor
```

---

## 6. Topic 接口

| Topic | 类型 | 方向 | 说明 |
|-------|------|------|------|
| `/perception/observation` | `sensor_msgs/Image` | → VLA | 相机观测图像 |
| `/vla/action_cmd` | `std_msgs/String` | VLA → 规划 | 7-DOF动作（JSON） |
| `/planning/trajectory` | `trajectory_msgs/JointTrajectory` | 规划 → 控制 | 100Hz关节轨迹 |
| `/control/joint_cmd` | `sensor_msgs/JointState` | 控制 → 机器人 | 力矩/位置指令 |
| `/safety/estop` | `std_msgs/Bool` | 安全 → 全部 | 急停信号 |

**VLA动作格式**（JSON）：
```json
{
  "action": [0.35, 0.12, 0.28, 0.0, 0.0, 0.0, 1.0],
  "latency_ms": 187.5,
  "timestamp": 1712345678.123,
  "mock": true
}
```
- `action[0:3]`：末端位置 (x, y, z)，单位 m
- `action[3:6]`：末端姿态 (rx, ry, rz)，单位 rad
- `action[6]`：夹爪开度，0=闭合，1=张开

---

## 7. 关键技术点

### 7.1 无锁环形缓冲区
- `lock_free_ring_buffer.hpp` — SPSC无锁队列，用于高吞吐数据传递
- 基于 `std::atomic` 的内存序优化，无锁无阻塞
- 适用于图像帧、控制指令等实时数据

### 7.2 1000Hz 实时控制
- 1ms 周期 PID 控制回路
- 积分抗饱和（anti-windup）
- 输出限幅保护
- Release 模式下 `-O3 -ffast-math` 优化

### 7.3 VLA 异步推理
- drop-old 队列策略：新帧到来时丢弃旧帧，保证最新性
- 独立推理线程，不阻塞 ROS 回调
- QoS：观测数据 KEEP_LAST(1)，动作指令 KEEP_LAST(10)

### 7.4 三层安全监控
1. **关节限位** — 机械硬限位检查
2. **速度限制** — 角速度超限检测
3. **力矩限制** — 输出力矩保护

---

## 8. 目录结构

```
embodied_rt/
├── CMakeLists.txt              # C++构建配置
├── package.xml                 # ROS2包描述
├── requirements.txt            # Python依赖
├── README.md                   # 本文档
├── config/
│   └── embodied_rt.yaml        # 系统参数配置
├── launch/
│   └── embodied_rt.launch.py   # 一键启动脚本
└── src/
    ├── vla_inference_node.py   # VLA推理节点（Python）
    ├── trajectory_generator.hpp  # 轨迹生成器声明
    ├── trajectory_generator.cpp  # 轨迹生成器实现
    ├── real_time_controller.cpp  # 实时PID控制器
    ├── safety_monitor.cpp        # 安全监控
    └── lock_free_ring_buffer.hpp # 无锁环形缓冲区
```

---

## 9. 常见问题

**Q: 编译时报 "MoveIt2 not found"？**
A: 正常现象。没有 MoveIt2 时 trajectory_generator 自动降级为线性插值模式，不影响运行。

**Q: Mock 模式下能看到什么效果？**
A: VLA 节点以 2Hz 发布模拟动作，轨迹生成器生成 200 个点的平滑轨迹，控制器以 1000Hz 输出 PID 力矩，安全监控全程检测。可以用 `ros2 topic echo` 查看各节点输出。

**Q: 怎么验证系统在正常工作？**
A: 运行 `ros2 topic hz /control/joint_cmd`，应该看到约 1000Hz 的发布频率。

**Q: 怎么接真实机器人？**
A: 把 `/control/joint_cmd` 接到机器人驱动节点（如 franka_ros2），把机器人的关节状态发布到 `/robot/joint_states` 供控制器订阅。
