cmake_minimum_required(VERSION 2.8.3)
project(velodyne_rotational_control)

# Find Gazebo
find_package(gazebo REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build our plugin
add_library(velodyne_rotational_control SHARED src/DiffDrivePlugin.cpp)
target_link_libraries(velodyne_rotational_control ${GAZEBO_LIBRARIES})

# install(DIRECTORY include/${PROJECT_NAME}/
#         DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )