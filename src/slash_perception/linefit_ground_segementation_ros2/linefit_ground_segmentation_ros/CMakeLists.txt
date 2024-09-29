cmake_minimum_required(VERSION 3.10) 
project(linefit_ground_segmentation_ros)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 添加优化标志
add_compile_options(-O3)

# 查找必要的包
find_package(ament_cmake_auto REQUIRED)
find_package(linefit_ground_segmentation REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PCL REQUIRED)

ament_auto_find_build_dependencies()

# 包含目录
include_directories(
  include
  ${linefit_ground_segmentation_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 添加可执行文件
ament_auto_add_executable(ground_segmentation_node 
  src/ground_segmentation_node.cc
)

ament_auto_add_executable(ground_segmentation_test_node 
  src/ground_segmentation_test_node.cc
)

# 链接库
target_link_libraries(ground_segmentation_node
  ${linefit_ground_segmentation_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(ground_segmentation_test_node
  ${linefit_ground_segmentation_LIBRARIES}
  ${PCL_LIBRARIES}
)

# 声明包依赖
ament_target_dependencies(ground_segmentation_node
  rclcpp
  sensor_msgs
  linefit_ground_segmentation
)

ament_target_dependencies(ground_segmentation_test_node
  rclcpp
  sensor_msgs
  linefit_ground_segmentation
)

ament_auto_package(INSTALL_TO_SHARE launch)

# QTCREATOR
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})