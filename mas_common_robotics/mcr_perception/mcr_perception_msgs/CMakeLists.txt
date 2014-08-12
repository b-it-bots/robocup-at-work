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
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxList.msg
    Face.msg
    FaceList.msg
    LaserScanSegment.msg
    LaserScanSegmentList.msg
    Object.msg
    ObjectList.msg
    Person.msg
    PersonList.msg
    Plane.msg
    PlanarPolygon.msg
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
