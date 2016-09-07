cmake_minimum_required(VERSION 2.8.3)
project(mcr_perception_msgs)

find_package(catkin REQUIRED 
  COMPONENTS
    geometry_msgs
    message_generation
    roscpp
    rospy
    sensor_msgs
    std_msgs
    visualization_msgs
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxList.msg
    Cavity.msg
    Face.msg
    FaceList.msg
    ImageList.msg
    LaserScanSegment.msg
    LaserScanSegmentList.msg
    MatchingErrorStamped.msg
    Object.msg
    ObjectList.msg
    Person.msg
    PersonList.msg
    Plane.msg
    PlanarPolygon.msg
    PointCloud2List.msg
    Blob.msg
    BlobList.msg
)

add_service_files(
  FILES
    AccumulateTabletopCloud.srv
    AnalyzeCloudColor.srv
    BaseScanLinearRegression.srv
    ClusterTabletopCloud.srv
    FindWorkspace.srv
    GetPersonList.srv
    GetObjectList.srv
    GetFaceName.srv
    GetFacialExpression.srv
    GetNearestObject.srv
    IsPersonInFront.srv
    MakeBoundingBoxes.srv
    PublishGoal.srv
    RecognizeObject.srv
    SetFaceName.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
    sensor_msgs
    std_msgs  
)

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs
    message_runtime
    sensor_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

### EXECUTABLES ###
add_executable(object_msg_visualizer_node
  ros/src/object_msg_visualizer_node.cpp
)
add_dependencies(object_msg_visualizer_node
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_gencpp
)
target_link_libraries(object_msg_visualizer_node
  ${catkin_LIBRARIES}
)

### TESTS
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  roslaunch_add_file_check(ros/launch)
endif()


### INSTALLS
install(
  TARGETS
    object_msg_visualizer_node 
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
