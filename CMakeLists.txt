# MatlabIO CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME "MatlabIO")
set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})
project(${PROJECT_NAME})

# find the dependencies
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
include(FindZLIB)

# include the dependencies
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories("include")

# GLOB the source files
file(GLOB SRC_FILES "src/*.cpp")
file(GLOB HDR_FILES "src/*.hpp")

# build as an executable
add_executable(${PROJECT_NAME} ${SRC_FILES})

#link
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZLIB_LIBRARIES})
