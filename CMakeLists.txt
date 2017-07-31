cmake_minimum_required(VERSION 2.8.3)
project(pcdfilter_pa)

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
  roscpp

  std_msgs
  sensor_msgs
  std_srvs
  message_generation

  tf
  laser_geometry

  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  parameter_pa
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_service_files(
  FILES
    PcdFilterPaFilter.srv
    PcdFilterPaCloud.srv
)
generate_messages(
  DEPENDENCIES
    std_msgs
    sensor_msgs
)

catkin_package(
  DEPENDS

  INCLUDE_DIRS
    include

  CATKIN_DEPENDS
    roscpp

    std_msgs
    sensor_msgs
    std_srvs
    message_runtime

    tf

    cv_bridge
    image_transport

    pcl_ros
    pcl_conversions

    parameter_pa

  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_ros
)


### exported libraries
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
  src/${PROJECT_NAME}_parameter.cpp
  src/${PROJECT_NAME}_filter.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_ros
  src/${PROJECT_NAME}_ros.cpp
  src/${PROJECT_NAME}_ros_parameter.cpp
  src/${PROJECT_NAME}_ros_throttle.cpp
  src/${PROJECT_NAME}_ros_filter.cpp
)
add_dependencies(${PROJECT_NAME}_ros
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_ros
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

### simple nodes
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
  src/${PROJECT_NAME}_node_parameter.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
  ${PROJECT_NAME}_generate_messages_cpp
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_ros
)

## Install ##
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.rviz"
)

