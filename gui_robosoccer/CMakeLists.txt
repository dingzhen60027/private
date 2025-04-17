cmake_minimum_required(VERSION 3.5)
project(gui_robosoccer)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fPIC -pthread)
endif()

set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(message_interface_robosoccer REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
#find_package(Boost 1.71.0 REQUIRED COMPONENTS thread)

#set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

#搜索头文件的位置
include_directories(
        include/
        ${CMAKE_CURRENT_BINARY_DIR}
)

file(GLOB PRO_FORM_DIR ui/*.ui)
file(GLOB PRO_RESOURCES_DIR resource/*.qrc)
file(GLOB PRO_INCLUDE_DIR include/*.h )
file(GLOB PRO_SOURCES_DIR src/*.cpp)

#qt头文件moc文件resource文件处理
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_executable(${PROJECT_NAME}
         ${PRO_FORM_DIR}
         ${PRO_RESOURCES_DIR}
         ${PRO_INCLUDE_DIR}
         ${PRO_SOURCES_DIR}
)
ament_target_dependencies(${PROJECT_NAME} 
         rclcpp message_interface_robosoccer std_msgs sensor_msgs OpenCV cv_bridge image_transport) # 添加依赖的 ROS2 库 ####Boost
target_link_libraries(${PROJECT_NAME}
        Qt5::Widgets
)
install(TARGETS ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
)

ament_package()
