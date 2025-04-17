cmake_minimum_required(VERSION 3.8)
project(vision_robosoccer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -pthread)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_interface_robosoccer REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

#连接大恒相机库-1
find_library(GXIAPI libgxiapi.so PATHS ${CMAKE_CURRENT_SOURCE_DIR}/lib REQUIRED)

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

ament_package()


include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}
)


add_executable(vision src/vision.cpp include/GxIAPI.h include/DxImageProc.h)
ament_target_dependencies(vision
                  rclcpp message_interface_robosoccer std_msgs sensor_msgs OpenCV cv_bridge image_transport)
#连接大恒相机库-2
target_link_libraries(vision ${GXIAPI})
install(TARGETS vision DESTINATION lib/${PROJECT_NAME})


add_executable(vision_processing src/vision_processing.cpp include/${PROJECT_NAME}/vision_processing.h)
ament_target_dependencies(vision_processing 
                  rclcpp message_interface_robosoccer std_msgs sensor_msgs OpenCV cv_bridge image_transport)
install(TARGETS vision_processing DESTINATION lib/${PROJECT_NAME})



#test
add_executable(vision_control_test src/vision_control_test.cpp)
ament_target_dependencies(vision_control_test 
                  rclcpp message_interface_robosoccer std_msgs)
install(TARGETS vision_control_test DESTINATION lib/${PROJECT_NAME})


add_executable(vision_display_test src/vision_display_test.cpp)
ament_target_dependencies(vision_display_test 
                  rclcpp message_interface_robosoccer std_msgs sensor_msgs OpenCV cv_bridge image_transport)
install(TARGETS vision_display_test DESTINATION lib/${PROJECT_NAME})
