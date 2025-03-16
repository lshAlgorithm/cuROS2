cmake_minimum_required(VERSION 3.5)
project(roscuda_vectoradd)

# Find CUDA
find_package(CUDA REQUIRED)

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Set RPATH
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# Add ROS 2 node
add_executable(vectoradd_node src/vectoradd_node.cpp)

# Link the ROS2 node to rclcpp
ament_target_dependencies(vectoradd_node rclcpp)

# Add CUDA source file as a shared library
cuda_add_library(vectoradd_cuda SHARED src/vectoradd.cu)

# Link CUDA library
target_link_libraries(vectoradd_node vectoradd_cuda)

# Install targets
install(TARGETS
  vectoradd_node
  vectoradd_cuda
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_package()
