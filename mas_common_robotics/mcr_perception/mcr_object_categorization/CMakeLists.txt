cmake_minimum_required(VERSION 2.8.3)
project(mcr_object_categorization)

find_package(catkin REQUIRED
  COMPONENTS
    geometry_msgs
    mcr_perception_msgs
    roscpp
    sensor_msgs
    std_srvs
    tf
    visualization_msgs
)
find_package(PCL 1.5 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(BLAS REQUIRED)
find_package(Boost REQUIRED
  COMPONENTS
    signals
    serialization
    thread
    wserialization
)
find_library(LOG4CPP_LIBRARY log4cpp)

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mcr_perception_msgs
    sensor_msgs
    std_srvs
    visualization_msgs
)

include_directories(
  common/include/${PROJECT_NAME}
  ros/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)


### LIBRARIES ###
add_library(johnson_apsp
  common/src/johnson_apsp/apsp_graph.cpp 
  common/src/johnson_apsp/apsp_node.cpp 
  common/src/johnson_apsp/edge.cpp
  common/src/johnson_apsp/graph.cpp 
  common/src/johnson_apsp/node.cpp
  common/src/johnson_apsp/prio_queue.cpp
)

add_library(neural_gas
  common/src/neural_gas/key_point_service_open_surf.cpp 
  common/src/neural_gas/key_point.cpp 
  common/src/neural_gas/mds.cpp 
  common/src/neural_gas/neural_gas_classifier.cpp 
  common/src/neural_gas/neural_gas.cpp  
  common/src/neural_gas/neural_gas_node.cpp 
  common/src/neural_gas/neural_gas_edge.cpp
  common/src/open_surf/fasthessian.cpp
  common/src/open_surf/integral.cpp 
  common/src/open_surf/ipoint.cpp
  common/src/open_surf/surf.cpp
  common/src/open_surf/utils.cpp   
)
target_link_libraries(neural_gas
  ${OpenCV_LIBRARIES}
)

add_library(auto_encoder
  common/src/logger.cpp  
  common/src/file_settings.cpp
  common/src/sparse_auto_encoder/sae_tool_box.cpp 
  common/src/sparse_auto_encoder/neuron.cpp 
  common/src/sparse_auto_encoder/layer.cpp 
  common/src/sparse_auto_encoder/neural_network.cpp 
  common/src/sparse_auto_encoder/auto_encoder.cpp 
  common/src/sparse_auto_encoder/stacked_auto_encoder.cpp 
)

add_library(object_categorization_geometric
  common/src/object_categorization_geometric.cpp 
  common/src/object_pseudo_categorization_geometric.cpp
)
target_link_libraries(object_categorization_geometric
  svm_linear_classifier
  svm_one_class_classifier
  ${PCL_LIBRARIES}
)

add_library(svm_classifier  
  common/src/svm/svm_classifier.cpp 
)
target_link_libraries(svm_classifier
  svm
)

add_library(svm_one_class_classifier  
  common/src/svm/svm_one_class_classifier.cpp
)
target_link_libraries(svm_one_class_classifier
  svm
)

  
add_library(svm_cascade  
  common/src/svm/svm_cascade.cpp
)
target_link_libraries(svm_cascade
  b_svm_classifier
)

add_library(b_svm_classifier  
  common/src/svm/b_svm_classifier.cpp
)
target_link_libraries(b_svm_classifier
  svm_classifier
)

add_library(svm_linear_classifier 
  common/src/svm/online_lib_linear/svm_linear_classifier.cpp
  common/src/svm/online_lib_linear/linear.cpp
  common/src/svm/online_lib_linear/tron.cpp
)
target_link_libraries(svm_linear_classifier
  ${BLAS_LIBRARIES}
)

  
add_library(object_geometric_service
  common/src/logger.cpp 
  common/src/object_geometric_service.cpp 
  common/src/file_settings.cpp 
  common/src/density_estimation.cpp 
  common/src/pca.cpp
  common/src/prob_neural_network/rtp_neural_network_ensemble.cpp   
  common/src/prob_neural_network/p_neural_network.cpp
  common/src/prob_neural_network/knn_classifier.cpp  
  common/src/prob_neural_network/hp_neural_network.cpp
  ros/src/toolbox_ros.cpp
)
target_link_libraries(object_geometric_service
  object_decomposition
  neural_gas
  rtp_neural_network_ensemble
  johnson_apsp
  ${LOG4CPP_LIBRARY}
  ${PCL_LIBRARIES}
)

add_library(rtp_neural_network_ensemble 
  common/src/file_settings.cpp 
  common/src/logger.cpp 
  common/src/prob_neural_network/rtp_neural_network_ensemble.cpp 
  common/src/prob_neural_network/rtp_neural_network.cpp 
  common/src/prob_neural_network/p_neural_network.cpp 
  common/src/prob_neural_network/hp_neural_network.cpp
)

add_library(object_decomposition 
  common/src/object_decomposition.cpp 
  common/src/emd/emd.cpp
)

### EXECUTABLES ###
add_executable(object_categorization_node 
  ros/src/object_categorization_node.cpp
  common/src/logger.cpp
)
add_dependencies(object_categorization_node 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(object_categorization_node
  object_categorization_geometric
  object_geometric_service
  auto_encoder
  svm_cascade
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(training_object_categorization 
  ros/tools/training_object_categorization.cpp
  common/src/logger.cpp
)
add_dependencies(training_object_categorization 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(training_object_categorization
  object_categorization_geometric
  object_geometric_service
  auto_encoder
  svm_cascade
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(convert_models_from_text_to_binary 
  ros/tools/convert_models_from_text_to_binary.cpp
  common/src/logger.cpp
)
target_link_libraries(convert_models_from_text_to_binary
  object_geometric_service
  ${Boost_LIBRARIES}
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
    object_categorization_node
    object_categorization_node
    training_object_categorization
    convert_models_from_text_to_binary
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  TARGETS
    auto_encoder
    object_geometric_service
    object_categorization_geometric
    rtp_neural_network_ensemble
    neural_gas object_decomposition
    johnson_apsp
    b_svm_classifier
    svm_linear_classifier
    svm_one_class_classifier
    svm_cascade
    svm_classifier
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)



install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)
