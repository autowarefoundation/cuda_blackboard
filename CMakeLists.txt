cmake_minimum_required(VERSION 3.14)
project(cuda_blackboard)

find_package(ament_cmake_auto REQUIRED)
find_package(CUDA)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wunused-function)
endif ()

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)
    ament_lint_auto_find_test_dependencies()

    # Header-only and CUDA-free, so it runs where CUDA is absent too (the CI containers).
    ament_add_gtest(test_publish_mailbox test/test_publish_mailbox.cpp)
    target_include_directories(test_publish_mailbox PRIVATE include)
endif()

if(NOT ${CUDA_FOUND})
  message(WARNING "cuda was not found. the cuda_blackboard package will not be built.")
  return()
endif()

ament_auto_find_build_dependencies()

include_directories(
    include
    SYSTEM
    ${CUDA_INCLUDE_DIRS}
)

ament_auto_add_library(cuda_blackboard SHARED
    src/cuda_blackboard.cpp
    src/cuda_image.cpp
    src/cuda_message_impl_detail.cpp
    src/cuda_pointcloud2.cpp
    src/cuda_blackboard_publisher.cpp
    src/cuda_blackboard_async_publisher.cpp
    src/cuda_blackboard_subscriber.cpp
    src/cuda_mem_pool_context.cpp
)

target_link_libraries(cuda_blackboard
    ${CUDA_LIBRARIES}
)
if(autoware_agnocast_wrapper_FOUND)
  autoware_agnocast_wrapper_setup(cuda_blackboard)
endif()

if(BUILD_TESTING)
    ament_auto_find_test_dependencies()

    # These need a CUDA device at test time (skipped without one).
    ament_add_gtest(test_cuda_blackboard_publisher test/test_cuda_blackboard_publisher.cpp)
    target_link_libraries(test_cuda_blackboard_publisher cuda_blackboard ${CUDA_LIBRARIES})

    ament_add_gtest(test_cuda_blackboard_async_publisher test/test_cuda_blackboard_async_publisher.cpp)
    target_link_libraries(test_cuda_blackboard_async_publisher cuda_blackboard ${CUDA_LIBRARIES})

    if(autoware_agnocast_wrapper_FOUND)
      ament_add_gtest(test_start_worker_thread test/test_start_worker_thread.cpp
        ENV ENABLE_AGNOCAST=1)
      target_link_libraries(test_start_worker_thread cuda_blackboard
        agnocast_cie_thread_configurator::agnocast_thread_configurator)
    endif()
endif()

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
