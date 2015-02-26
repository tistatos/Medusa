cmake_minimum_required(VERSION 2.8.7)

set(PROJECT_NAME medusa)
project(${PROJECT_NAME} C CXX)
include(ExternalProject)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_CXX_DIR ${PROJECT_SOURCE_DIR}/src)
set(PROJECT_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
set(PROJECT_EXT_DIR ${PROJECT_SOURCE_DIR}/external)

###############
# PCL Settings
###############

set(PCL_DIR "/path/to/PCLConfig.cmake")
find_package(PCL REQUIRED COMPONENTS common)
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


include_directories(
        ${PROJECT_INCLUDE_DIR}
)


add_executable(test src/main.cpp)
target_link_libraries(test ${PCL_COMMON_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES})
