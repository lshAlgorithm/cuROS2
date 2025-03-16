cmake_minimum_required(VERSION 3.8)
project(cuGEMM)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(CUDA REQUIRED)
find_package(rclcpp REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Set RPATH
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# Add the directory containing the shared library to the linker search path
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/cuGEMM)

# Add ROS 2 node
add_executable(gemm_node src/gemm_node.cpp)

# Link the ROS2 node to rclcpp
ament_target_dependencies(gemm_node rclcpp)

# Add CUDA source file as a shared library
cuda_add_library(cugemm SHARED src/gemm.cu)

# Link CUDA library
target_link_libraries(gemm_node cugemm)

# Install targets
install(TARGETS
  gemm_node
  cugemm
  DESTINATION lib/${PROJECT_NAME})

# Export dependencies
ament_package()
