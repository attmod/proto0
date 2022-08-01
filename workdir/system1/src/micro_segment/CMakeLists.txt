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
find_package(OpenCV REQUIRED)

#if (VTK_VERSION VERSION_LESS "8.90.0")
#  include(${VTK_USE_FILE})
#endif()



list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)
INCLUDE_DIRECTORIES(
        ${ICUB_CONTRIB_DIRS}
        ${OPENCV_INCLUDE_DIR}
)

# we use thrift for rpc defintion
yarp_add_idl(IDL_GEN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/rpc.thrift)
add_executable(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp ${IDL_GEN_FILES})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(${PROJECT_NAME} PRIVATE _USE_MATH_DEFINES)

if (VTK_VERSION VERSION_GREATER_EQUAL "8.90.0")
  vtk_module_autoinit(TARGETS ${PROJECT_NAME} MODULES ${VTK_LIBRARIES})
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${YARP_LIBRARIES} ICUB::iKin ${OpenCV_LIBS} ${VTK_LIBRARIES})

install(TARGETS ${PROJECT_NAME} DESTINATION bin)

