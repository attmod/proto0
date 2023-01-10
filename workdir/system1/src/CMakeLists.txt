# Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
#                    Istituto Italiano di Tecnologia, Genova
# Licence: GPLv2 (please see LICENSE file)


## set the current project (and binary) name to the name of the directory
# get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
# string(REPLACE " " "_" ProjectId ${ProjectId})
# project(${ProjectId})

## old, in case you need to freeze the name
set( PROJECT_NAME system1-solid ) # this is a manual override, use PROJECT() instead

find_package(YARP REQUIRED COMPONENTS os dev sig math pcl)
find_package(ICUB REQUIRED COMPONENTS iKin)
find_package(ICUBcontrib REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)
INCLUDE_DIRECTORIES(
        ${ICUB_CONTRIB_DIRS}
        ${OPENCV_INCLUDE_DIR}
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# we use thrift for rpc defintion
yarp_add_idl(IDL_GEN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.thrift)
#add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp)

add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${IDL_GEN_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

target_link_libraries(${PROJECT_NAME} PRIVATE ${YARP_LIBRARIES} ICUB::iKin ${OpenCV_LIBS} ${PCL_LIBRARIES} Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME} DESTINATION bin)


set( PROJECT_NAME gazebo_teleporter ) # this is a manual override, use PROJECT() instead

find_package(YARP REQUIRED COMPONENTS os dev sig math)
find_package(ICUB REQUIRED COMPONENTS iKin)
find_package(ICUBcontrib REQUIRED)
find_package(Gazebo REQUIRED)

set(PLUGIN_MOVER ${PROJECT_NAME})
add_library(${PLUGIN_MOVER} SHARED gazebo_teleporter.cpp)
target_include_directories(${PLUGIN_MOVER} PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_MOVER} PRIVATE ${GAZEBO_LIBRARIES} YARP::YARP_os)
install(TARGETS ${PLUGIN_MOVER} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)
