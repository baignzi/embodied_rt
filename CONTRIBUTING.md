# Contributing to EmbodiedRT

感谢您对 EmbodiedRT 的关注！本文档将帮助您快速了解如何参与项目开发。

## 环境准备

### 必需依赖

- Ubuntu 22.04
- ROS2 Humble（或 Iron）
- `colcon` 构建工具
- `clang-format`（用于代码格式化）

### 可选依赖

- MoveIt2（完整模式需要）
- Google Test（运行单元测试需要）
- Doxygen（生成 API 文档需要）

### 快速搭建

```bash
# 创建工作空间
mkdir -p ~/embodied_rt_ws/src
cd ~/embodied_rt_ws/src

# 克隆仓库
git clone https://github.com/your-org/embodied_rt.git
cd ..

# 安装 ROS2 依赖
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 加载环境
source install/setup.bash
```

## 开发流程

1. **Fork 仓库** 并克隆到本地。
2. **创建功能分支**：`git checkout -b feat/your-feature-name`。
3. **编写代码** 并确保通过所有测试。
4. **格式化代码**：`find src test -name '*.cpp' -o -name '*.hpp' | xargs clang-format -i`。
5. **提交更改**（遵循 [Conventional Commits](#提交信息规范)）。
6. **推送分支**：`git push origin feat/your-feature-name`。
7. **创建 Pull Request** 到 `main` 分支。

## 代码风格

### C++

- 基于 **Google C++ Style Guide**，使用项目提供的 `.clang-format` 配置。
- 缩进：4 个空格（不使用 Tab）。
- 行宽：100 列。
- 命名规范：
  - 类型名：`PascalCase`（如 `TrajectoryPlanner`）
  - 变量名：`snake_case`（如 `joint_positions_`）
  - 私有成员：尾部下划线（如 `pid_params_`）
  - 常量：`kPascalCase` 或全大写下划线（如 `kMaxVelocity`）
  - 宏：全大写下划线（如 `HAS_MOVEIT`）
- 头文件使用 `#pragma once` 或 include guard。
- 优先使用 `const` 和引用传递，避免不必要的拷贝。

### Python

- 遵循 **PEP 8**。
- 使用 4 空格缩进。
- 行宽：100 列。
- 类型注解推荐但非强制。

## 提交信息规范

采用 [Conventional Commits](https://www.conventionalcommits.org/) 规范：

```
<type>(<scope>): <subject>

<body>

<footer>
```

### Type 说明

| Type | 含义 |
|------|------|
| `feat` | 新功能 |
| `fix` | 修复 Bug |
| `docs` | 仅文档变更 |
| `style` | 不影响代码含义的格式变更（空格、分号等） |
| `refactor` | 代码重构，既不修复 Bug 也不添加功能 |
| `perf` | 性能优化 |
| `test` | 添加或修正测试 |
| `chore` | 构建过程或辅助工具的变更 |
| `ci` | CI 配置变更 |

### 示例

```
feat: add low-pass filter to PID derivative term

- Add `prev_deriv` to PIDState
- Introduce `derivative_lpf_alpha` parameter
- Apply filter before adding to total output

Closes #11
```

## 测试

### 运行单元测试

```bash
cd ~/embodied_rt_ws
colcon test --packages-select embodied_rt
colcon test-result --verbose
```

### 手动测试

Mock 模式无需 GPU，适合快速验证：

```bash
ros2 launch embodied_rt embodied_rt.launch.py
```

验证 1000 Hz 控制频率：

```bash
ros2 topic hz /control/joint_cmd
```

## 目录结构约定

```
embodied_rt/
├── src/            # C++ 源代码
├── test/           # gtest 单元测试
├── launch/         # ROS2 启动脚本
├── config/         # YAML 参数配置
├── .github/        # GitHub Actions CI
├── CMakeLists.txt  # 构建配置
└── package.xml     # ROS2 包描述
```

## 报告问题

提交 Issue 时请包含：

1. **环境信息**：OS、ROS2 版本、编译器版本。
2. **复现步骤**：最小可复现的命令或代码片段。
3. **预期行为** vs **实际行为**。
4. **日志输出**：相关终端输出或错误信息。

## 安全与实时性注意事项

- **实时控制节点**（`real_time_controller`、`safety_monitor`）运行在 1000 Hz，避免在其中进行：
  - 动态内存分配（`new`、`malloc`、STL 容器扩容）
  - 阻塞 I/O 或系统调用
  - 复杂数学运算未经过性能测试
- 任何涉及安全监控的修改必须经过充分的边界条件测试。

## 许可证

本项目采用 MIT 许可证。提交代码即表示您同意将您的贡献在 MIT 许可证下发布。
