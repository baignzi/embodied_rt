# EmbodiedRT 代码改进计划（V2）

每日改进一项，从高优先级到低优先级。完成后将 `done` 标记为 ✅，并提交到远端。

## 待改进项

| #  | 优先级 | 模块                     | 改进内容                                                                                     | done |
| -- | --- | ---------------------- | ------------------------------------------------------------------------------------------ | ---- |
| 1  | P0  | trajectory_generator   | 修复 fallback_blend_ratio=0 时除以零：denom=blend*(1-blend) 为 0 导致 NaN，需加保护分支或调整 clamp 下界         | ✅    |
| 2  | P0  | real_time_controller   | 订阅 /safety/estop，收到急停后立即停止输出轨迹指令（当前控制器完全不知道急停）                              | ✅    |
| 3  | P1  | real_time_controller   | dt 动态计算：从定时器或消息时间戳计算实际 dt，替代硬编码 0.001                                                | ✅    |
| 4  | P1  | real_time_controller   | current_state_ 不应直接赋值为 target，应从 /joint_states 反馈读取（否则 PID 积分/微分项失效）                       | ✅    |
| 5  | P1  | config/yaml            | 补全 kff 和 deriv_filter_alpha 参数到 YAML；移除未使用的 frequency 参数                                          | ✅    |
| 6  | P1  | trajectory_generator   | on_action 回调加锁保护 current_joins_，防止数据竞争                                                        | ✅    |
| 7  | P1  | benchmark_node.py      | 输出路径用 os.path.join(tempfile.gettempdir(), ...) 替代硬编码 /tmp/；vla_receive_times 字典加 maxlen 防内存泄漏        | ✅    |
| 8  | P2  | launch                  | RViz2 和 robot_state_publisher 加 IfCondition，headless 环境可跳过                                         | ✅    |
| 9  | P2  | lock_free_ring_buffer  | push() 覆写路径注释说明 SPSC 语义 tradeoff，或改为 drop-new 策略                                             | ✅    |
| 10 | P2  | vla_inference_node.py  | _obs_callback 加图像编码检查，非 RGB8 时转换或警告                                                         | ✅    |
| 11 | P2  | safety_monitor         | 健康日志从 DEBUG 改为 INFO + 节流（每 N 次打印一次）                                                      | ✅    |
| 12 | P2  | trajectory_generator   | 浮点循环改整数迭代消除累积误差：for (int i=0; i*dt<=duration; ++i)                                          | ✅    |
| 13 | P2  | README.md              | 更新目录结构，补全 test/、pid_controller.hpp、benchmark_node.py、.github/、.clang-format                        | ✅    |
| 14 | P2  | CMakeLists.txt         | 显式 find_package(rcl_interfaces)，不依赖传递依赖                                                        | ✅    |
| 15 | P3  | project                | 添加 CHANGELOG.md + CONTRIBUTING.md                                                                    | ✅    |
| 16 | P3  | project                | 添加 .clang-tidy 静态分析配置                                                                            | ❌    |
| 17 | P3  | IMPROVEMENT_PLAN       | 标记旧计划 #11 和 #14 为完成（代码已实现）                                                                 | ❌    |

## 进度

- 已完成：15 / 17
- 下一项：#16 添加 .clang-tidy 静态分析配置
