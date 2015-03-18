cmake_minimum_required(VERSION 2.8.7)

set(PROJECT_NAME medusa)
project(${PROJECT_NAME} C CXX)
include(ExternalProject)

find_package(Threads REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
set(PROJECT_CXX_DIR ${PROJECT_SOURCE_DIR}/src)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "You must create a build folder and call cmake .. from there!")
endif()


###############
# PCL Settings
###############

set(PCL_DIR "/path/to/PCLConfig.cmake")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${EIGEN_INCLUDE_DIRS})




##################
# OpenCV Settings
##################

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

##################
# freenect Settings
##################

find_package(libfreenect REQUIRED )
include_directories(${libfreenect_INCLUDE_DIRS})
link_directories(${FREENECT_LIBRARY_DIRS})


##################
# targets
##################
include_directories(
        ${PROJECT_INCLUDE_DIR}
)



add_executable(dependencies_test src/main.cpp)
target_link_libraries(dependencies_test ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})


add_executable(renderMesh src/renderMesh.cpp)
target_link_libraries(renderMesh ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})


add_executable(PCD_test src/loadPCD.cpp)
target_link_libraries(PCD_test
 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})

 add_executable(pcl_for_file src/pclFunctionsForFile.cpp)
target_link_libraries(pcl_for_file
 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})

 add_executable(mainRender src/mainRender.cpp)
target_link_libraries(mainRender
 ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})

add_library(kinect src/kinect.cpp)

set(ALL_LIBS ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES} kinect)



