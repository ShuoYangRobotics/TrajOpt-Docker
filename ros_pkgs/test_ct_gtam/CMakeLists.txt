cmake_minimum_required(VERSION 3.10)
project(test_ct_gtsam)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
SET (CMAKE_CXX_STANDARD 11)
SET (CMAKE_CXX_STANDARD_REQUIRED ON)
SET (CMAKE_CXX_FLAGS "-O3")
set (CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -fpermissive")

## find non-ros pkgs
find_package(Eigen3 REQUIRED)

# control toolbox
find_package(ct_core REQUIRED)
find_package(ct_optcon REQUIRED)
find_package(ct_rbd REQUIRED)
find_package(ct_models REQUIRED)

# casadi symbolic computation
find_package(casadi REQUIRED)

# gtsam
# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation
find_package(GTSAM REQUIRED) 
 
set(CMAKE_INCLUDE_CURRENT_DIR ON)
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/
  ${CASADI_INCLUDE_DIR}
)
# this requires 
# sudo apt-get install python3-dev
# sudo apt-get install python3-matplotlib
set(PLOTTING_ENABLED ON)

## find ros packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)
#  DEPENDS system_lib
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
)

# build a info data path
set(INFO_DIR "${CMAKE_CURRENT_SOURCE_DIR}/info")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/info/infoDir.h.in ${CMAKE_CURRENT_SOURCE_DIR}/info/infoDir.h)


# test control toolbox initial installation
# add_executable(test_ct ${CMAKE_CURRENT_SOURCE_DIR}/src/test_ct.cpp)
# target_link_libraries( test_ct
#                           ${catkin_LIBRARIES} 
#                           ct_core
#                           )
# # test using control toolbox to solve iLQR
# add_executable(ct_ilqr ${CMAKE_CURRENT_SOURCE_DIR}/src/ct_ilqr.cpp)
# target_link_libraries( ct_ilqr
#                           ${catkin_LIBRARIES} 
#                           ct_core
#                           ct_optcon
#                           )                          


# add_executable(test_gtsam ${CMAKE_CURRENT_SOURCE_DIR}/src/test_gtsam.cpp)
# target_link_libraries( test_gtsam
#                           ${catkin_LIBRARIES} 
#                           gtsam
#                           )
add_executable(test_noisemodel ${CMAKE_CURRENT_SOURCE_DIR}/src/test_noisemodel.cpp)
target_link_libraries( test_noisemodel
                          ${catkin_LIBRARIES} 
                          gtsam
                          CppUnitLite
                          )