cmake_minimum_required(VERSION 2.8.3)
project(mcr_algorithms)
find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs mcr_perception_msgs)
find_package(PCL 1.6 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    geometry_msgs 
    mcr_perception_msgs
    message_runtime 
    sensor_msgs 
  INCLUDE_DIRS 
    common/include/
  LIBRARIES 
    mcr_controller
    mcr_geometry
    mcr_machine_learning
    mcr_segmentation
)

include_directories(
  common/include/
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(mcr_segmentation 
  common/src/segmentation/laserscan_segmentation.cpp
  common/src/segmentation/pointcloud_segmentation.cpp
)
add_dependencies(mcr_segmentation 
  ${catkin_EXPORTED_TARGETS}
)

add_library(mcr_geometry 
  common/src/geometry/conversions.cpp
  common/src/geometry/geometric_distances.cpp
  common/src/geometry/geometric_properties.cpp	
)

add_library(mcr_wrapper 
  common/src/wrapper/pcl_wrapper.cpp
)

add_library(mcr_statistics 
  common/src/statistics/means.cpp
  common/src/statistics/minmax.cpp
)

add_library(mcr_projections 
  common/src/projections/pointcloud_projections.cpp
)

add_library(mcr_io 
  common/src/io/file_access.cpp
)

add_library(mcr_machine_learning 
  common/src/machine_learning/random_trees.cpp
  common/src/machine_learning/adaboost.cpp
  common/src/machine_learning/svm_trainer.cpp
  common/src/machine_learning/svm_classifier.cpp
)

target_link_libraries(mcr_machine_learning
    ${OpenCV_LIBRARIES}
    ${Boost_LIBRARIES}
    svm
)

add_library(mcr_data_fitting 
  common/src/data_fitting/circle_2d.cpp
)

add_library(mcr_neural_networks
  common/src/neural_networks/self_organizing_map.cpp
)

add_library(mcr_controller
  common/src/controller/pd_controller.cpp
  common/src/controller/pi_controller.cpp
)


install(DIRECTORY common/include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
	TARGETS
		mcr_geometry 
		mcr_segmentation
		mcr_controller
		mcr_machine_learning
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
