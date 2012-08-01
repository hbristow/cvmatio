# build the project
if(BUILD_EXECUTABLE)
    set(SRC_FILES MatlabIO.cpp main.cpp)
    add_executable(${PROJECT_NAME} ${SRC_FILES})
else()
    set(SRC_FILES MatlabIO.cpp)
    add_library(${PROJECT_NAME} SHARED ${SRC_FILES})
endif()

#link
#find_library(opencv_imgproc_LIBRARIES opencv_imgproc /u/hbristow/dev/lib)
find_library(opencv_core_LIBRARIES NAMES opencv_core PATHS /u/hbristow/dev/lib NO_DEFAULT_PATH)

target_link_libraries(${PROJECT_NAME}   ${Boost_LIBRARIES} 
#                                        ${OpenCV_LIBS} 
                                        ${ZLIB_LIBRARIES}
                                        ${opencv_core_LIBRARIES}
)

# install
install(TARGETS ${PROJECT_NAME} RUNTIME DESTINATION ${PROJECT_SOURCE_DIR}/bin
                                LIBRARY DESTINATION ${PROJECT_SOURCE_DIR}/lib
)
