# Copyright (c) 2022 Social Cognition in Human-Robot Interaction,
#                    Istituto Italiano di Tecnologia, Genova
# Licence: GPLv2 (please see LICENSE file)


## set the current project (and binary) name to the name of the directory
get_filename_component(ProjectId ${CMAKE_CURRENT_SOURCE_DIR} NAME)
string(REPLACE " " "_" ProjectId ${ProjectId})
project(${ProjectId})

## old, in case you need to freeze the name
# set( PROJECT_NAME image-to-image-test ) # this is a manual override, use PROJECT() instead

find_package(YARP REQUIRED COMPONENTS os dev sig math)
find_package(ICUB REQUIRED COMPONENTS iKin)
find_package(ICUBcontrib REQUIRED)
find_package(Gazebo REQUIRED)

set(PLUGIN_MOVER ${PROJECT_NAME})
add_library(${PLUGIN_MOVER} SHARED model_mover.cpp)
target_include_directories(${PLUGIN_MOVER} PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PLUGIN_MOVER} PRIVATE ${GAZEBO_LIBRARIES} YARP::YARP_os)
install(TARGETS ${PLUGIN_MOVER} LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT shlib)
