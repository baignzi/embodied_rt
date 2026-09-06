# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added

- Added low-pass filter to PID derivative term for measurement noise suppression.
- Added Doxygen-style comments to all header files and generated API documentation.
- Added GitHub Actions CI workflow for automated build and test.
- Added gtest unit tests for PID controller and trajectory resampling.
- Added `.clang-format` configuration (Google style, 100 columns, 4 spaces).
- Added `CHANGELOG.md` and `CONTRIBUTING.md`.

### Changed

- Refactored PID controller: separated parameters (`PIDParams`) from runtime state (`PIDState`).
- Refactored real-time controller to pre-allocate `JointState` message at 1000 Hz, eliminating string reallocation.
- Parameterized linear fallback trajectory with configurable scale, duration, blend ratio, and time step.
- Replaced `static` local variables with member variables and improved `const` correctness across C++ sources.
- Replaced `__has_include` with CMake-controlled `HAS_MOVEIT` macro in trajectory generator.

### Fixed

- Fixed e-stop latching behavior in safety monitor: once triggered, e-stop remains active until explicitly reset.
- Fixed hard-coded `dt` in safety monitor: now dynamically calculated from actual loop timing.
- Fixed MoveIt2 detection to avoid link failures when headers exist but libraries do not.

## [1.0.0] - 2024-08-28

### Added

- Initial release of EmbodiedRT.
- End-to-end VLA (Vision-Language-Action) model-driven 7-DOF robot arm real-time control framework.
- Four core ROS2 nodes:
  - `vla_inference_node` (Python, 2–5 Hz): VLA model inference with mock/real mode support.
  - `trajectory_generator` (C++): RRT* via MoveIt2 or standalone linear interpolation fallback.
  - `real_time_controller` (C++): 1000 Hz PID torque control with anti-windup and output clamping.
  - `safety_monitor` (C++): 1000 Hz joint limit, velocity, and torque monitoring with e-stop.
- Lock-free SPSC ring buffer for high-throughput real-time data passing.
- ROS2 launch file for one-click startup.
- YAML-based system parameter configuration.
- RViz2 integration for real-time trajectory visualization.
