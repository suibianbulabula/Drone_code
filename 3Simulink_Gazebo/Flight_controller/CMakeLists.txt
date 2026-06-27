cmake_minimum_required(VERSION 3.5)
project(flight_controller)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

set(SITL_GAZEBO_BUILD /home/dbh/PX4/PX4-Autopilot/PX4-Autopilot/Tools/sitl_gazebo/build)

#添加头文件搜索路径
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${GAZEBO_INCLUDE_DIRS}
    ${PROTOBUF_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${SITL_GAZEBO_BUILD}
    ${SITL_GAZEBO_BUILD}/msgs
    /usr/include/gazebo-11/gazebo/msgs
    /home/dbh/c_library_v2                    # MAVLink 根目录
)

#列出所有源文件（带 src/ 前缀）
set(SOURCES
    src/main.cpp
    src/sensor.cpp
    src/attitude_control.cpp
    src/vision_control.cpp
    src/position_control.cpp
    src/planning.cpp
    src/globals.cpp
    src/my_mavlink.cpp
)

add_executable(flight_controller ${SOURCES})

target_link_libraries(flight_controller
    ${GAZEBO_LIBRARIES}
    gazebo_transport
    gazebo_common
    ${PROTOBUF_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${SITL_GAZEBO_BUILD}/libmav_msgs.so
    ${SITL_GAZEBO_BUILD}/libsensor_msgs.so
    Boost::system
    tbb
    pthread
)

target_compile_options(flight_controller PRIVATE -Wno-address-of-packed-member)
