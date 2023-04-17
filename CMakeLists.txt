cmake_minimum_required(VERSION 3.10.2)
project(tracked_robot_tools)

option(BUILD_PYTHON "Build Python SWIG module" ON)
set(PYTHON_VERSION $ENV{ROS_PYTHON_VERSION} CACHE STRING "Version of Python libraries to search for")

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  nav_msgs
  roscpp
  tf2_geometry_msgs
)

if(BUILD_PYTHON)
  catkin_python_setup()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES tracked_robot_flipper_postures tracked_robot_kinematics tracked_robot_odometry
  CATKIN_DEPENDS cras_cpp_common nav_msgs roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(tracked_robot_kinematics src/kinematics.cpp)
add_dependencies(tracked_robot_kinematics ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracked_robot_kinematics ${catkin_LIBRARIES})

add_library(tracked_robot_odometry src/skid_steer_odometry.cpp)
add_dependencies(tracked_robot_odometry ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracked_robot_odometry tracked_robot_kinematics ${catkin_LIBRARIES})

add_library(tracked_robot_flipper_postures src/flipper_postures.cpp)
add_dependencies(tracked_robot_flipper_postures ${catkin_EXPORTED_TARGETS})
target_link_libraries(tracked_robot_flipper_postures ${catkin_LIBRARIES})

install(TARGETS tracked_robot_flipper_postures tracked_robot_kinematics tracked_robot_odometry
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Compile the Python library if required
if(BUILD_PYTHON)
  add_subdirectory(python)
endif()

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_skid_steer_odom test/test_skid_steer_odometry.cpp)
  target_link_libraries(test_skid_steer_odom tracked_robot_odometry)
endif()