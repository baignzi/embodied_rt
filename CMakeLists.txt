cmake_minimum_required(VERSION 3.8)
project(embodied_rt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -Wall -Wextra")

# 实时优化：Release模式优化
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNDEBUG")
    set_source_files_properties(
        src/real_time_controller.cpp
        PROPERTIES COMPILE_FLAGS "-O3 -march=native -ffast-math")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# --- 实时控制器 ---
add_executable(real_time_controller src/real_time_controller.cpp)
target_link_libraries(real_time_controller
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
)
target_include_directories(real_time_controller PRIVATE src)

# --- 安全监控 ---
add_executable(safety_monitor src/safety_monitor.cpp)
target_link_libraries(safety_monitor
    rclcpp::rclcpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
)
target_include_directories(safety_monitor PRIVATE src)

# --- 轨迹生成器 ---
# MoveIt2 为可选依赖，代码中用 __has_include 检测
# 无 MoveIt2 时自动降级为 standalone 线性插值模式
find_package(moveit_ros_planning_interface QUIET)
find_package(tf2 QUIET)
find_package(tf2_geometry_msgs QUIET)

add_executable(trajectory_generator src/trajectory_generator.cpp)
target_include_directories(trajectory_generator PRIVATE src)
target_link_libraries(trajectory_generator
    rclcpp::rclcpp
    trajectory_msgs::trajectory_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
)

# 尝试链接 MoveIt2（三种方式兼容不同版本）
if(moveit_ros_planning_interface_FOUND)
    # 方式1: 用变量链接（最兼容）
    if(DEFINED moveit_ros_planning_interface_LIBRARIES)
        target_link_libraries(trajectory_generator
            ${moveit_ros_planning_interface_LIBRARIES})
        target_include_directories(trajectory_generator PRIVATE
            ${moveit_ros_planning_interface_INCLUDE_DIRS})
        target_compile_definitions(trajectory_generator PRIVATE HAS_MOVEIT=1)
        message(STATUS "EmbodiedRT: MoveIt2 linked via LIBRARIES variable")
    # 方式2: ament_target_dependencies
    elseif(DEFINED ament_cmake_core_DIR)
        ament_target_dependencies(trajectory_generator
            moveit_ros_planning_interface tf2 tf2_geometry_msgs)
        target_compile_definitions(trajectory_generator PRIVATE HAS_MOVEIT=1)
        message(STATUS "EmbodiedRT: MoveIt2 linked via ament_target_dependencies")
    # 方式3: 都不行就不链接，代码走 standalone
    else()
        message(STATUS "EmbodiedRT: MoveIt2 found but no link target — standalone fallback")
    endif()
else()
    message(STATUS "EmbodiedRT: MoveIt2 NOT found — standalone linear fallback")
endif()

# --- VLA推理节点 (Python) ---
install(PROGRAMS src/vla_inference_node.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME vla_inference_node
)

# --- 安装可执行文件 ---
install(TARGETS
    real_time_controller
    safety_monitor
    trajectory_generator
    DESTINATION lib/${PROJECT_NAME}
)

# --- 配置文件 ---
install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

# --- Launch 文件 ---
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
