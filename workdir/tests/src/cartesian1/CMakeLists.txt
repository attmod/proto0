cmake_minimum_required(VERSION 3.12)

find_package(YARP)

add_executable(cartesian1 cartesian1.cpp)
target_compile_definitions(cartesian1 PRIVATE _USE_MATH_DEFINES)
target_link_libraries(cartesian1 ${YARP_LIBRARIES})
install(TARGETS cartesian1 DESTINATION bin)

