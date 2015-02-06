cmake_minimum_required(VERSION 2.8.11)
set(PROJECT_NAME medusa)
project($(PROJECT_NAME) C CXX)
include(ExternalProject)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_CXX_DIR ${PROJECT_SOURCE_DIR}/src)
set(PROJECT_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
set(PROJECT_EXT_DIR ${PROJECT_SOURCE_DIR}/external)

# ---------------
# PCL settings
# ---------------








 ExternalProject_Add( pcl
  URL ${PROJECT_EXT_DIR}/pcl/
  CMAKE_CACHE_ARGS
  -DBUILD_visualization:BOOL=OFF
  -DBUILD_2d:BOOL=OFF
  -DBUILD_geometry:BOOL=OFF
  -DBUILD_keypoints:BOOL=OFF
  -DBUILD_tracking:BOOL=OFF
  -DBUILD_recognition:BOOL=OFF
  -DWITH_VTK:BOOL=OFF
  )
# ---------------
# OpenCV settings
# ---------------

ExternalProject_Add( opencv
  URL ${PROJECT_EXT_DIR}/opencv/
)

# ---------------
# libfreenect settings
# ---------------
set(OFF CACHE BOOL "")

ExternalProject_Add( libfreenect
 URL ${PROJECT_EXT_DIR}/libfreenect/
 CMAKE_CACHE_ARGS -DBUILD_EXAMPLES:BOOL=OFF
)


set(ext_libs
  ${CMAKE_SOURCE_DIR}/external/libfreenect/include/
  ${CMAKE_SOURCE_DIR}/external/opencv/include/
)


include_directories(
        ${ext_libs}
        ${PROJECT_INCLUDE_DIR}
)