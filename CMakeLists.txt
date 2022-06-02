cmake_minimum_required(VERSION 3.0.2)
project(carrot_planner)
set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV  REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(catkin REQUIRED
  COMPONENTS
    angles
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_geometry_msgs
    tf2_ros
    roslib
    geometry_msgs
    visualization_msgs
)

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES carrot_planner
  CATKIN_DEPENDS
    angles
    base_local_planner
    costmap_2d
    nav_core
    pluginlib
    roscpp
    tf2
    tf2_ros
    geometry_msgs
    visualization_msgs
)


file(GLOB lib_source  src/carrot_planner.cpp 
                      src/Astar.cpp
                      src/OccMapTransform.cpp
                      src/read_yaml.cpp)

add_library(carrot_planner ${lib_source})
add_dependencies(carrot_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(carrot_planner
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    yaml-cpp)


install(TARGETS carrot_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


