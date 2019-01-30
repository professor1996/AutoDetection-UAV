cmake_minimum_required(VERSION 2.8.3)
project(gf_detection)

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
pcl_ros
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
pcl_ros
     message_runtime
     gf_perception
)

include_directories(
  include
  /home/pengrui/Documents/GaoFen_Drone/src/airsim-ros/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

list(APPEND SOURCEFILES
#   src/airsim_node.cpp
)

add_executable(front_cam_detection
src/front_cam_detection.cpp
src/num_detect.cpp
    # ${SOURCEFILES}
)


target_link_libraries(front_cam_detection
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )


add_executable(record
src/record.cpp
src/num_detect.cpp
    # ${SOURCEFILES}
)


target_link_libraries(record
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )



add_executable(down
src/image_down.cpp
src/image_matching.cpp
    # ${SOURCEFILES}
)


target_link_libraries(down
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )

add_executable(depth
src/image_depth2.cpp
    # ${SOURCEFILES}
)


target_link_libraries(depth
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )



add_executable(front
src/image_matching_front.cpp
    # ${SOURCEFILES}
)


target_link_libraries(front
   ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
 )
