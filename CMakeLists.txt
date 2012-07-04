# MatlabIO CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(MatlabIO)

# -----------------------------------------------
# USER-DEFINED VARIABLES
option(BUILD_EXECUTABLE "Build as executable to test functionality" OFF)
option(BUILD_DOC        "Build documentation with Doxygen"          ON)



set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

# find the dependencies
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
include(FindZLIB)

# include the dependencies
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories("include")

# build the project
if(BUILD_EXECUTABLE)
    set(SRC_FILES src/MatlabIO.cpp src/main.cpp)
    add_executable(MatlabIO ${SRC_FILES})
else()
    set(SRC_FILES src/MatlabIO.cpp)
    add_library(MatlabIO SHARED ${SRC_FILES})
endif()

#link
target_link_libraries(MatlabIO ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZLIB_LIBRARIES})

# install the targets
install(TARGETS MatlabIO RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib)

# build the doxygen pdf
if(BUILD_DOC)
    find_package(Doxygen)
    configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in ${PROJECT_SOURCE_DIR}/doc/Doxyfile @ONLY)
    add_custom_target(doc ALL ${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/doc/Doxyfile)
    add_custom_command(COMMAND make 
        OUTPUT ${PROJECT_SOURCE_DIR}/doc/latex
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doc/latex
    )
endif()
