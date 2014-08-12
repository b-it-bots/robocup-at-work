cmake_minimum_required(VERSION 2.8.3)
project(polyclipping)
find_package(catkin REQUIRED
  COMPONENTS
    mk
)

add_custom_target(
  build_polyclipping_lib ALL
  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball

  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}

  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/clipper_ver4.10.0/cpp/build/libpolyclipping.so.4.0.0 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libpolyclipping.so.4.0.0

  COMMAND cmake -E create_symlink ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libpolyclipping.so.4.0.0 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libpolyclipping.so.4

  COMMAND cmake -E create_symlink ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libpolyclipping.so.4.0.0 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/libpolyclipping.so
  
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/clipper_ver4.10.0/cpp/clipper.hpp ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}/clipper.hpp
)

catkin_package(
    LIBRARIES polyclipping
)

add_library(polyclipping)
add_dependencies(polyclipping 
  build_polyclipping_lib
)
set_target_properties(polyclipping PROPERTIES LINKER_LANGUAGE CXX)

install(TARGETS polyclipping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY build/clipper_ver4.10.0/cpp/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
  PATTERN "cpp_agg" EXCLUDE
  PATTERN "cpp_cairo" EXCLUDE
  PATTERN "cpp_console" EXCLUDE
  PATTERN "cpp_opengl" EXCLUDE
)
