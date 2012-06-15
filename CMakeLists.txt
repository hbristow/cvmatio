# MatlabIO CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
set(PROJECT_NAME "MatlabIO")
project(${PROJECT_NAME})

# find the dependencies
find_package(Boost)
find_package(OpenCV)

# include the dependencies
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# GLOB the source files
file(GLOB SRC_FILES "src/*.cpp")
file(GLOB HDR_FILES "src/*.hpp")

# build as an executable
add_executable(${PROJECT_NAME} ${SRC_FILES})

#link
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${OPENCV_LIBRARIES})
