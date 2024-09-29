cmake_minimum_required(VERSION 3.10)
project(linefit_ground_segmentation)

# 设置 C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 添加优化标志
add_compile_options(-O3)

# 查找必要的包
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)

ament_auto_find_build_dependencies()

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 添加库
add_library(${PROJECT_NAME} SHARED
  src/ground_segmentation.cc
  src/viewer.cc
  src/segment.cc
  src/bin.cc
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  ${Boost_LIBRARIES}
)

# 安装头文件
install(
  DIRECTORY include/ground_segmentation
  DESTINATION include
)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(PCL Boost)

ament_auto_package()

# QTCREATOR
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})