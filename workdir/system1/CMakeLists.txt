cmake_minimum_required(VERSION 3.12)
project (system1)

find_package(YARP REQUIRED COMPONENTS sig cv)
find_package(ICUBcontrib REQUIRED)
# FIND_PACKAGE(GTK2)

# IF(GTK2_FOUND)
#     message("GTK FOUND ")
# ELSE(GTK2_FOUND)
#     message("GTK not found")
# ENDIF(GTK2_FOUND)


list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix()


# we keep the second part in src
SET(SYSTEM1LIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src/include)
add_subdirectory(src)

add_subdirectory(scripts)

#icubcontrib_finalize_export(system1) # this caused suddenly an error with install
icubcontrib_set_default_prefix()
#icubcontrib_add_uninstall_target() # disabled for now
