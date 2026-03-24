cmake_minimum_required(VERSION 3.16)
project(anima_pallas_ros2)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(pallas_core_runtime
  src/attitude_bootstrap.cpp
  src/cloud_ingest.cpp
  src/core_runtime.cpp
  src/scan_sampler.cpp
  src/strapdown_tracker.cpp
  src/surfel_volume.cpp)

target_include_directories(pallas_core_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pallas_core_runtime PUBLIC cxx_std_17)
target_link_libraries(pallas_core_runtime Eigen3::Eigen)
ament_target_dependencies(pallas_core_runtime
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

add_library(pallas_ct_runtime
  src/ct_runtime.cpp
  src/timed_pose_spline.cpp)

target_include_directories(pallas_ct_runtime PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pallas_ct_runtime PUBLIC cxx_std_17)
target_link_libraries(pallas_ct_runtime pallas_core_runtime Eigen3::Eigen)
ament_target_dependencies(pallas_ct_runtime
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

add_executable(anima_pallas_core_node src/pallas_core_node.cpp)
target_compile_features(anima_pallas_core_node PUBLIC cxx_std_17)
target_link_libraries(anima_pallas_core_node pallas_core_runtime Eigen3::Eigen)
ament_target_dependencies(anima_pallas_core_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

add_executable(anima_pallas_ct_node src/pallas_ct_node.cpp)
target_compile_features(anima_pallas_ct_node PUBLIC cxx_std_17)
target_link_libraries(anima_pallas_ct_node pallas_ct_runtime Eigen3::Eigen)
ament_target_dependencies(anima_pallas_ct_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

install(
  TARGETS pallas_core_runtime pallas_ct_runtime anima_pallas_core_node anima_pallas_ct_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
