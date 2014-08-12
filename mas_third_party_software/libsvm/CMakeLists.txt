cmake_minimum_required(VERSION 2.8.3)
project(libsvm)
find_package(catkin REQUIRED 
  COMPONENTS
    mk
)

add_custom_target(
  build_libsvm_lib ALL
  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball

  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}

  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/libsvm-3.18/libsvm.so.2 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libsvm.so
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/libsvm-3.18/libsvm.so.2 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libsvm.so.2
  
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/libsvm-3.18/svm.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/svm.h
)

catkin_package(
    LIBRARIES svm
)

add_library(svm)
add_dependencies(svm 
  build_libsvm_lib
)
set_target_properties(svm PROPERTIES LINKER_LANGUAGE CXX)

list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS build_libsvm_lib)

install(TARGETS svm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY build/libsvm-3.18/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "java" EXCLUDE
  PATTERN "python" EXCLUDE
  PATTERN "svm-toy" EXCLUDE
  PATTERN "tools" EXCLUDE
  PATTERN "windows" EXCLUDE
  PATTERN "matlab" EXCLUDE
)
