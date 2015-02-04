cmake_minimum_required(VERSION 2.8.8)
set(PROJECT_NAME medusa)
project($(PROJECT_NAME) C CXX)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_CXX_DIR ${PROJECT_SOURCE_DIR}/src)
set(PROJECT_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
set(PROJECT_EXT_DIR ${PROJECT_SOURCE_DIR}/external)

set(BUILD_visualization OFF CACHE BOOL "")
set(BUILD_2d OFF CACHE BOOL "")
set(BUILD_geometry OFF CACHE BOOL "")
set(BUILD_geometry OFF CACHE BOOL "")


#add_subdirectory(
#  ${PROJECT_EXT_DIR}/opencv/
#)
#add_subdirectory(
#  ${PROJECT_EXT_DIR}/libfreenect/
#)

add_subdirectory(
  ${PROJECT_EXT_DIR}/pcl/
)

set(ext_libs
#  ${CMAKE_SOURCE_DIR}/external/libfreenect/include/
#  ${CMAKE_SOURCE_DIR}/external/opencv/include/
)

include_directories(
        ${ext_libs}
        ${PROJECT_INCLUDE_DIR}
)
