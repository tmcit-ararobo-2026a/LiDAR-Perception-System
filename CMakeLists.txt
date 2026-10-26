cmake_minimum_required(VERSION 3.18)
project(lidar_perception_system LANGUAGES C CXX)
option(BUILD_RUNTIME_NODES "Build perception runtime nodes" ON)
option(BUILD_GPU_NODES "Build CUDA detector (CPU guard can be built separately)" ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CUDA_STANDARD 17)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# CPU-only geometry is shared by the new tracker and local synthetic checks.
add_library(opening_tracking_core STATIC src/opening_tracker.cpp src/bucket_bottom.cpp)
target_include_directories(opening_tracking_core PUBLIC include)
target_link_libraries(opening_tracking_core PUBLIC Eigen3::Eigen)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MovingBucketTrack.msg"
  "msg/BucketBottom.msg"
  DEPENDENCIES std_msgs builtin_interfaces geometry_msgs)

if(BUILD_RUNTIME_NODES)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
add_executable(track_freshness_guard_node src/track_freshness_guard_node.cpp)
ament_target_dependencies(track_freshness_guard_node rclcpp sensor_msgs geometry_msgs tf2_ros)
target_link_libraries(track_freshness_guard_node
  nlohmann_json::nlohmann_json "${cpp_typesupport_target}")
install(TARGETS track_freshness_guard_node DESTINATION lib/${PROJECT_NAME})

add_executable(bucket_opening_tracker src/bucket_opening_tracker_node.cpp)
ament_target_dependencies(bucket_opening_tracker
  rclcpp sensor_msgs geometry_msgs diagnostic_msgs visualization_msgs tf2 tf2_ros tf2_msgs)
target_link_libraries(bucket_opening_tracker opening_tracking_core
  nlohmann_json::nlohmann_json "${cpp_typesupport_target}")
install(TARGETS bucket_opening_tracker DESTINATION lib/${PROJECT_NAME})
if(BUILD_GPU_NODES)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)
add_library(cuda_grid STATIC src/cuda_grid.cu)
target_include_directories(cuda_grid PUBLIC include)
target_link_libraries(cuda_grid PUBLIC CUDA::cudart)
set_target_properties(cuda_grid PROPERTIES CUDA_SEPARABLE_COMPILATION ON POSITION_INDEPENDENT_CODE ON)

add_executable(moving_bucket_detector src/moving_bucket_detector.cpp)
target_include_directories(moving_bucket_detector PRIVATE include)
ament_target_dependencies(moving_bucket_detector
  rclcpp sensor_msgs geometry_msgs diagnostic_msgs visualization_msgs tf2 tf2_ros tf2_msgs)
target_link_libraries(moving_bucket_detector cuda_grid Eigen3::Eigen nlohmann_json::nlohmann_json)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(moving_bucket_detector ${cpp_typesupport_target})

install(TARGETS moving_bucket_detector DESTINATION lib/${PROJECT_NAME})
endif()
endif()

include(CTest)
if(BUILD_TESTING)
  add_executable(support_tracker_test test/support_tracker_test.cpp)
  target_include_directories(support_tracker_test PRIVATE include)
  target_link_libraries(support_tracker_test Eigen3::Eigen)
  add_test(NAME support_tracker_test COMMAND support_tracker_test)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/bucket_bottom_test.cpp")
    add_executable(bucket_bottom_test test/bucket_bottom_test.cpp)
    target_link_libraries(bucket_bottom_test opening_tracking_core)
    add_test(NAME bucket_bottom_test COMMAND bucket_bottom_test)
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/opening_tracker_test.cpp")
    add_executable(opening_tracker_test test/opening_tracker_test.cpp)
    target_link_libraries(opening_tracker_test opening_tracking_core)
    add_test(NAME opening_tracker_test COMMAND opening_tracker_test)
  endif()
  if(BUILD_RUNTIME_NODES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/track_freshness_guard_test.py")
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(track_freshness_guard_test test/track_freshness_guard_test.py
      ENV TRACK_GUARD_EXECUTABLE=$<TARGET_FILE:track_freshness_guard_node>
      TIMEOUT 30)
  endif()
  if(BUILD_RUNTIME_NODES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/bucket_bottom_ros_test.py")
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(bucket_bottom_ros_test test/bucket_bottom_ros_test.py
      ENV OPENING_TRACKER_EXECUTABLE=$<TARGET_FILE:bucket_opening_tracker>
          OPENING_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/bucket_opening.json
      TIMEOUT 45)
  endif()
  if(BUILD_RUNTIME_NODES AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/opening_tracker_ros_test.py")
    find_package(ament_cmake_pytest REQUIRED)
    ament_add_pytest_test(opening_tracker_ros_test test/opening_tracker_ros_test.py
      ENV OPENING_TRACKER_EXECUTABLE=$<TARGET_FILE:bucket_opening_tracker>
          TRACK_GUARD_EXECUTABLE=$<TARGET_FILE:track_freshness_guard_node>
          OPENING_CONFIG=${CMAKE_CURRENT_SOURCE_DIR}/config/bucket_opening.json
      TIMEOUT 45)
  endif()
endif()

install(PROGRAMS scripts/replay_static_tf.py scripts/benchmark_opening.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime)
ament_package()
