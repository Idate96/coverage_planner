cmake_minimum_required(VERSION 3.5.1)
set (CMAKE_CXX_STANDARD 20)
project(coverage_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  grid_map_ros
  grid_map_cv
  grid_map_core
  grid_map_msgs
  grid_map_visualization
  grid_map_rviz_plugin
  m545_path_utils
  cv_bridge
  )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES coverage_planner
  CATKIN_DEPENDS  
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_library(${PROJECT_NAME}
  src/CellDecompBsd.cpp
  src/Cell.cpp
  src/Graph.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(image_to_gridmap
  src/image_to_gridmap_node.cpp
  src/ImageToGridmap.cpp
)

target_link_libraries(image_to_gridmap
  ${catkin_LIBRARIES}
)

add_executable(load_map_bag
  test/load_map_bag.cpp
)

target_link_libraries(load_map_bag
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS scripts/image_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark executables and/or libraries for installation
install(
  TARGETS 
    ${PROJECT_NAME}
    image_to_gridmap
    load_map_bag
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(
  DIRECTORY 
    include/${PROJECT_NAME}/
    config
    data
    launch
    rviz
    scripts
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test 
        test/bsd_test.cpp
        test/test_cells.cpp
        test/graph_test.cpp
        test/decomp_test_map.cpp
        )
      


  target_include_directories(${PROJECT_NAME}-test PRIVATE
    include
  )

  target_include_directories(${PROJECT_NAME}-test SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
  )

  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

  # catkin_add_gtest(${PROJECT_NAME}-test-vis 
  #   test/decomp_test.cpp)

  # target_include_directories(${PROJECT_NAME}-test-vis PRIVATE
  #   include
  # )

  # target_include_directories(${PROJECT_NAME}-test-vis SYSTEM PUBLIC
  #   ${catkin_INCLUDE_DIRS}
  #   ${EIGEN3_INCLUDE_DIR}
  # )

  # target_link_libraries(${PROJECT_NAME}-test-vis ${PROJECT_NAME})

  # catkin_add_gtest(decomp_map-test 
  #     test/decomp_test_map.cpp
  #     )
    
  # target_include_directories(decomp_map-test PRIVATE
  #   include)
  

  # target_include_directories(decomp_map-test SYSTEM PUBLIC
  #   ${catkin_INCLUDE_DIRS}
  #   ${EIGEN3_INCLUDE_DIR}
  # )

  # target_link_libraries(decomp_map-test ${PROJECT_NAME})
  

  ###################
  ## Code_coverage ##
  ###################
  find_package(cmake_code_coverage QUIET)
  if(cmake_code_coverage_FOUND)
    add_gtest_coverage(
      TEST_BUILD_TARGETS
        ${PROJECT_NAME}-test
    )
  endif()
endif()
