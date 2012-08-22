# MatlabIO CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(cvmatio)

# -----------------------------------------------
# USER-DEFINED VARIABLES
option(BUILD_EXECUTABLE "Build as executable to test functionality" OFF)
option(BUILD_DOC        "Build documentation with Doxygen"          ON)

# --------------------
# CATKIN
set(BUILD_CATKIN OFF)
if (catkin_FOUND)
    set(BUILD_CATKIN ON)
    catkin_stack()
    catkin_project(cvmatio
                   DEPENDS Boost OpenCV
                   INCLUDE_DIRS include
                   LIBRARIES cvmatio
    )
endif()

# --------------------
# COMMON

# include all warning types
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Werror -pedantic")

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING
        "Choose the type of build. Options are: None, Debug, Release, RelWithDebInfo, MinSizeRel"
    FORCE)
endif()

# find the dependencies
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
include(FindZLIB)

# include the dependencies
include_directories(SYSTEM ${OPENCV_INCLUDE_DIRS}
                           ${Boost_INCLUDE_DIRS}
                           ${ZLIB_INCLUDE_DIR})
include_directories("include")

# build the project
add_subdirectory(src)

# build the doxygen pdf
if(BUILD_DOC)
    find_package(Doxygen)
    configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in ${PROJECT_SOURCE_DIR}/doc/Doxyfile @ONLY)
    add_custom_target(cvmatio_doc ALL ${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/doc/Doxyfile)
endif()

# configuration summary
message("")
message("---------------------------------------------")
message(${PROJECT_NAME} " Configuration Summary")
message("")
message("Building with ECTO/Catkin:     ${BUILD_CATKIN}")
message("Build as executable:           ${BUILD_EXECUTABLE}")
message("Build with documentation:      ${BUILD_DOC}")
message("Build type:                    ${CMAKE_BUILD_TYPE}")
message("")
message("---------------------------------------------")
