cmake_minimum_required(VERSION 3.0.2)
project(real)

#add_definitions(-std=c++11)
### Compile as C++11, supported in ROS Kinetic and newer
#set(CMAKE_CXX_STANDARD 11)

#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pthread")
#add_definitions("-DENABLE_SSE")
#set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${SSE_FLAGS} -msse4")
add_definitions(-std=c++11)
#set(CMAKE_CXX_FLAGS "-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  message_generation
  mavros_msgs
  geometry_msgs
  cv_bridge
  image_transport
  sensor_msgs
)

find_package (Eigen3 REQUIRED)
find_package (OpenCV 4 REQUIRED)
find_package (Sophus REQUIRED )

generate_messages(DEPENDENCIES std_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS
roscpp std_msgs message_generation sensor_msgs)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Sophus_INCLUDE_DIRS}
)

add_executable(FSM src/FSM.cpp)
target_link_libraries(FSM
${catkin_LIBRARIES}
)

add_executable(Failsafe src/Failsafe.cpp)
target_link_libraries(Failsafe
${catkin_LIBRARIES}
)

add_executable(Aruco src/Aruco.cpp)
target_link_libraries(Aruco
${OpenCV_LIBRARIES}
${catkin_LIBRARIES}
${Sophus_LIBRARIES}
)

#add_executable(KalmanFilter src/KalmanFilter.cpp)
#target_link_libraries(KalmanFilter
#${OpenCV_LIBRARIES}
#${catkin_LIBRARIES}
#${Sophus_LIBRARIES}
#)

add_executable(Yolo src/Yolo.cpp
                    src/utils/run_yolo.cpp)
target_link_libraries(Yolo
   ${OpenCV_LIBRARIES}
   ${catkin_LIBRARIES})