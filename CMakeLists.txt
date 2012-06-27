# MatlabIO CMakeLists.txt
cmake_minimum_required(VERSION 2.8)
project(MatlabIO)
set(PROJECT_ROOT ${PROJECT_SOURCE_DIR})

# find the dependencies
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
include(FindZLIB)

# add Doxygen support
find_package(Doxygen)
configure_file(${PROJECT_SOURCE_DIR}/doc/Doxyfile.in ${PROJECT_SOURCE_DIR}/doc/Doxyfile @ONLY)
add_custom_target(doc ALL ${DOXYGEN_EXECUTABLE} ${PROJECT_SOURCE_DIR}/doc/Doxyfile)

# include the dependencies
include_directories(${OPENCV_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${ZLIB_INCLUDE_DIR})
include_directories("include")

# GLOB the source files
file(GLOB SRC_FILES "src/*.cpp")
file(GLOB HDR_FILES "src/*.hpp")

# build as an executable
add_executable(MatlabIO ${SRC_FILES})

#link
target_link_libraries(MatlabIO ${Boost_LIBRARIES} ${OpenCV_LIBS} ${ZLIB_LIBRARIES})

# build the doxygen pdf
add_custom_command(COMMAND make 
    OUTPUT ${PROJECT_SOURCE_DIR}/doc/latex
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doc/latex
)

# install the targets
install(FILES ${PROJECT_SOURCE_DIR}/build/MatlabIO DESTINATION ${PROJECT_SOURCE_DIR}/bin)
install(FILES ${PROJECT_SOURCE_DIR}/doc/latex/refman.pdf DESTINATION ${PROJECT_SOURCE_DIR}/doc/latex/MatlabIO.pdf)
