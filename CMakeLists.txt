cmake_minimum_required(VERSION 3.16)
project(eskf_gnss_imu_localization)

set(IMU_CALIBRATION_CONFIG_PATH ${CMAKE_CURRENT_SOURCE_DIR}/params/imu.yaml)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(message_filters REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(GeographicLib REQUIRED)
find_package(yaml-cpp REQUIRED)

add_subdirectory(modules)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/gnss_subscriber.cpp
  src/imu_subscriber.cpp
  src/math_utils.cpp
  src/eskf.cpp
)
target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
  ${EIGEN3_INCLUDE_DIR}
  ${GeographicLib_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_node 
  rclcpp
  sensor_msgs
  geometry_msgs
  message_filters
)
target_link_libraries(${PROJECT_NAME}_node
  Eigen3::Eigen
  ${GeographicLib_LIBRARIES}
  opengl_viewer
  ${YAML_CPP_LIBRARIES}
)

target_compile_definitions(${PROJECT_NAME}_node PUBLIC
  IMU_CALIBRATION_CONFIG_PATH="${IMU_CALIBRATION_CONFIG_PATH}"
)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
