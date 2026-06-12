cmake_minimum_required(VERSION 3.14)
project(cuda_blackboard)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda was not found. the cuda_blackboard package will not be built.")
  return()
endif()

ament_auto_find_build_dependencies()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()


if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(
    include
    SYSTEM
    ${CUDA_INCLUDE_DIRS}
)

ament_auto_add_library(cuda_blackboard SHARED
    src/cuda_blackboard.cpp
    src/cuda_image.cpp
    src/cuda_pointcloud2.cpp
    src/cuda_blackboard_publisher.cpp
    src/cuda_blackboard_subscriber.cpp
    src/cuda_mem_pool_context.cpp
)

target_link_libraries(cuda_blackboard
    ${CUDA_LIBRARIES}
)

####

ament_auto_add_library(cuda_blackboard_publisher_example SHARED
  src/cuda_blackboard_publisher_node.cpp
)

target_link_libraries(cuda_blackboard_publisher_example
  cuda_blackboard
)

rclcpp_components_register_node(cuda_blackboard_publisher_example
  PLUGIN "cuda_blackboard::CudaBlackboardPublisherNode"
  EXECUTABLE cuda_blackboard_publisher_node
)

ament_auto_add_library(cuda_blackboard_subscriber_example SHARED
  src/cuda_blackboard_subscriber_node.cpp
)

target_link_libraries(cuda_blackboard_subscriber_example
  cuda_blackboard
)

rclcpp_components_register_node(cuda_blackboard_subscriber_example
  PLUGIN "cuda_blackboard::CudaBlackboardSubscriberNode"
  EXECUTABLE cuda_blackboard_subscriber_node
)

# This example contains a .cu source, so it is built with cuda_add_library (legacy FindCUDA).
# cuda_add_library does not get ament_auto's automatic dependency wiring, so ROS dependencies
# must be listed explicitly via ament_target_dependencies.
cuda_add_library(cuda_blackboard_subscriber_with_stream_example SHARED
  src/cuda_blackboard_subscriber_node_with_stream.cu
)

ament_target_dependencies(cuda_blackboard_subscriber_with_stream_example
  rclcpp
  rclcpp_components
  sensor_msgs
)

target_link_libraries(cuda_blackboard_subscriber_with_stream_example
  cuda_blackboard
  ${CUDA_LIBRARIES}
)

rclcpp_components_register_node(cuda_blackboard_subscriber_with_stream_example
  PLUGIN "cuda_blackboard::CudaBlackboardSubscriberNodeWithStream"
  EXECUTABLE cuda_blackboard_subscriber_node_with_stream
)

# cuda_add_library targets are not tracked by ament_auto, so install this one explicitly.
install(TARGETS cuda_blackboard_subscriber_with_stream_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()

# Set ROS_DISTRO macros
set(ROS_DISTRO $ENV{ROS_DISTRO})
if(${ROS_DISTRO} STREQUAL "rolling")
    add_compile_definitions(ROS_DISTRO_ROLLING)
elseif(${ROS_DISTRO} STREQUAL "foxy")
    add_compile_definitions(ROS_DISTRO_FOXY)
elseif(${ROS_DISTRO} STREQUAL "galactic")
    add_compile_definitions(ROS_DISTRO_GALACTIC)
elseif(${ROS_DISTRO} STREQUAL "humble")
    add_compile_definitions(ROS_DISTRO_HUMBLE)
endif()
