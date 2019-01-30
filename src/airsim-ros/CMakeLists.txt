cmake_minimum_required(VERSION 2.8.3)
project(airsim-ros)

find_package(Threads REQUIRED)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-std=c++14 -ggdb -Wall -O3 -pthread ${CMAKE_CXX_FLAGS}")
# -Wshadow -Wold-style-cast -Wno-documentation
# -Wextra -Wstrict-aliasing -Wunreachable-code -Wcast-qual -Wctor-dtor-privacy \
#                 -Wdisabled-optimization -Wformat=2 -Winit-self -Wmissing-include-dirs -Wswitch-default \
#                  -Woverloaded-virtual -Wredundant-decls  -Wstrict-overflow=5 -Wswitch-default -Wundef \
# -Wno-variadic-macros -Wno-parentheses -Wno-unused-function -Wno-unused  -fdiagnostics-show-option \
set(BUILD_PLATFORM "x64")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)

find_package(OpenCV 3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  tf2
  nav_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  message_generation
  gf_perception
)


################################################
## Declare ROS messages, services and actions ##
################################################

# find_package(opencv 3 required)

# # msg 文件
# # Generate messages in the 'msg' folder
# add_message_files(
#    FILES
#    Message1.msg
# )
# # msg 依赖
# generate_messages(
#     DEPENDENCIES
#     std_msgs
# )
# # Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
# )
# # Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
# )


###########
## Build ##
###########

catkin_package(
    # INCLUDE_DIRS include   #声明给其它package的include路径
    # LIBRARIES ${PRIJECT_NAME}    #声明给其它package的库
    CATKIN_DEPENDS
    roscpp
    rospy
    tf
    tf2
    nav_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
    cv_bridge
    image_transport
    message_runtime
    gf_perception
)

include_directories(
  include
  include/AirLib
  include/MavLinkCom
  include/rpc
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
message("*****${CMAKE_CURRENT_SOURCE_DIR}/lib*********")
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib)

list(APPEND SOURCEFILES
  src/airsim_node.cpp
)

add_executable(airsim_node
    src/main.cpp
    ${SOURCEFILES}
)


target_link_libraries(airsim_node
    AirLib
    rpc
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )

add_executable(2airsim_node
    src/main3.cpp
    ${SOURCEFILES}
)


target_link_libraries(2airsim_node
    AirLib
    rpc
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )

add_executable(airsim_test
 src/airsim_test.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_test
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(airsim_image_front_rgb
 src/airsim_image_front_rgb.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_image_front_rgb
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)
add_executable(airsim_image_down
 src/airsim_image_down.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_image_down
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(airsim_image_front_depth
 src/airsim_image_front_depth.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_image_front_depth
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(airsim_imu
 src/airsim_imu.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_imu
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

# add_executable(airsim_gps
#  src/airsim_gps.cpp
#  ${SOURCEFILES}
# )


# target_link_libraries(airsim_gps
#  AirLib
#  rpc
# ${catkin_LIBRARIES}
#  ${OpenCV_LIBS}
# )


add_executable(airsim_barometer
 src/airsim_barometer.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_barometer
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)


add_executable(airsim_magnetic
 src/airsim_magnetic.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_magnetic
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)

add_executable(airsim_control
 src/airsim_control2.cpp
 ${SOURCEFILES}
)


target_link_libraries(airsim_control
 AirLib
 rpc
${catkin_LIBRARIES}
 ${OpenCV_LIBS}
)
