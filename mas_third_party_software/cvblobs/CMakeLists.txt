cmake_minimum_required(VERSION 2.8.3)
project(cvblobs)

find_package(catkin REQUIRED 
  COMPONENTS 
    mk
)

find_package(OpenCV REQUIRED)


add_custom_target(
  build_cvblobs_lib ALL
  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball

  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}

  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/libblob.a ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/BlobContour.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/BlobContour.h
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/blob.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/blob.h
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/BlobLibraryConfiguration.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/BlobLibraryConfiguration.h
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/BlobOperators.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/BlobOperators.h
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/BlobProperties.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/BlobProperties.h
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/BlobResult.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/BlobResult.h
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/cvblobs8.3_linux/ComponentLabeling.h ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/ComponentLabeling.h
)

catkin_package(
    LIBRARIES blob
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(blob STATIC)
add_dependencies(blob
  build_cvblobs_lib
)
set_target_properties(blob PROPERTIES LINKER_LANGUAGE CXX)

install(DIRECTORY build/cvblobs8.3_linux/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN "testBlobs" EXCLUDE
)

install(TARGETS blob
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
