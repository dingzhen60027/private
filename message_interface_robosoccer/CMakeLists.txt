cmake_minimum_required(VERSION 3.8)
project(message_interface_robosoccer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)

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

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArrayThreshold.msg"
  "msg/PointCalibration.msg"
  "msg/Point.msg"
  "msg/Image.msg"
  "msg/PoseRobots.msg"
  "msg/Pose.msg"
  "msg/Velocity.msg"
  "msg/VelocityRobots.msg"
  "msg/DecisionVar.msg"
  DEPENDENCIES sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
