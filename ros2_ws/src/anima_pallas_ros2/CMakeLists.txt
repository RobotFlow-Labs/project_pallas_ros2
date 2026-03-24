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

add_library(pallas_core
  src/imu_buffer.cpp
  src/local_map.cpp
  src/motion_integrator.cpp
  src/pipeline.cpp
  src/point_parser.cpp)

target_include_directories(pallas_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pallas_core PUBLIC cxx_std_17)
target_link_libraries(pallas_core PUBLIC Eigen3::Eigen)
ament_target_dependencies(pallas_core
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

add_executable(anima_pallas_node src/pallas_node.cpp)
target_compile_features(anima_pallas_node PUBLIC cxx_std_17)
target_link_libraries(anima_pallas_node PRIVATE pallas_core Eigen3::Eigen)
ament_target_dependencies(anima_pallas_node
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros)

install(
  TARGETS pallas_core anima_pallas_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
