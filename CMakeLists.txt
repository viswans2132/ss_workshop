cmake_minimum_required(VERSION 2.8.3)
project(ss_workshop)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  message_generation
)

find_package(Eigen REQUIRED)

add_service_files(
  FILES
  ControlStatus.srv  # Your custom service file here
  SwitchBodyCommand.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other message dependencies if any
)

catkin_package(
  INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}
  LIBRARIES  ss_pid_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs std_msgs message_runtime
  DEPENDS Eigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

install(DIRECTORY models/meshes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)


add_library(ss_pid_controller
  src/libs/pid_controller.cpp
)

add_library(ss_vel_controller
  src/libs/vel_controller.cpp
)

target_link_libraries(ss_pid_controller ${catkin_LIBRARIES})
add_dependencies(ss_pid_controller ${catkin_EXPORTED_TARGETS})


target_link_libraries(ss_vel_controller ${catkin_LIBRARIES})
add_dependencies(ss_vel_controller ${catkin_EXPORTED_TARGETS})


add_executable(ss_pid_node src/nodes/pid_node.cpp)
add_dependencies(ss_pid_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ss_pid_node
  ss_pid_controller ${catkin_LIBRARIES})

add_executable(ss_vel_node src/nodes/vel_node.cpp)
add_dependencies(ss_vel_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(ss_vel_node
  ss_vel_controller ${catkin_LIBRARIES})

install(TARGETS ss_pid_controller ss_vel_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ss_pid_node ss_vel_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

catkin_install_python(PROGRAMS
  src/scripts/spot_cbf.py
  src/scripts/laser_cbf_shafterx4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

