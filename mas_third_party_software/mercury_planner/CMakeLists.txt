cmake_minimum_required(VERSION 2.8.3)
project(mercury_planner)
find_package(catkin REQUIRED COMPONENTS
    mk
)

add_custom_target(
  mercury_planner ALL
  COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tarball
  
  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/${PROJECT_NAME}

  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/Mercury-fixed/seq-sat-mercury/src/preprocess/preprocess ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/${PROJECT_NAME}/
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/Mercury-fixed/seq-sat-mercury/src/search/downward-1 ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_LIB_DESTINATION}/${PROJECT_NAME}/

  #patch mercury planner
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/patch/actions.py ${PROJECT_SOURCE_DIR}/build/Mercury-fixed/seq-sat-mercury/src/translate/pddl/actions.py
  COMMAND cmake -E copy ${PROJECT_SOURCE_DIR}/patch/effects.py ${PROJECT_SOURCE_DIR}/build/Mercury-fixed/seq-sat-mercury/src/translate/pddl/effects.py

  COMMAND cmake -E make_directory ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/pddl
  COMMAND cmake -E copy_if_different ${PROJECT_SOURCE_DIR}/build/Mercury-fixed/seq-sat-mercury/src/translate/graph.py ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/pddl/
  COMMAND cmake -E copy_directory ${PROJECT_SOURCE_DIR}/build/Mercury-fixed/seq-sat-mercury/src/translate/pddl ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${PROJECT_NAME}/pddl/
)

catkin_package()
