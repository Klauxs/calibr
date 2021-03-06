cmake_minimum_required( VERSION 2.8 )
project( calibr )

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11")

find_package( catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    camera_models
    )

find_package( Sophus REQUIRED )
find_package(Ceres REQUIRED)

include_directories( "/usr/include/eigen3" 
    ${catkin_INCLUDE_DIRS} 
    ${Sophus_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    )

catkin_package()

add_executable( calibr 
    src/calibr_node.cpp    
    src/parameter.cpp
    src/project_factor.cpp
    src/pose_parameterization.cpp
    src/data_class.cpp
    )

target_link_libraries( calibr 
    ${catkin_LIBRARIES} 
    ${Sophus_LIBRARIES}    
    ${CERES_LIBRARIES}
    )