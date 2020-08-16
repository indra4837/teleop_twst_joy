cmake_minimum_required(VERSION 3.0.2)
project(teleop_twist_joy)

find_package(
  catkin REQUIRED COMPONENTS 
  geometry_msgs 
  roscpp 
  sensor_msgs 
  joy 
  message_generation
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs roscpp sensor_msgs joy message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/TeleopTwistJoy)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(
  teleop_twist_joy_node 
  src/teleop_twist_joy_node.cpp
  src/TeleopTwistJoy.cpp
)

# add_message_files(
#  FILES
#  joy_custom.msg
# )

# add_dependencies(
#  teleop_twist_joy_node
#  src/TeleopTwistJoy.cpp
#  src/teleop_twist_joy_node.cpp
# )

target_link_libraries(teleop_twist_joy_node ${PROJECT_NAME} ${catkin_LIBRARIES})
set_target_properties(teleop_twist_joy_node
  PROPERTIES OUTPUT_NAME teleop_node PREFIX "")

install(TARGETS ${PROJECT_NAME} teleop_twist_joy_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
