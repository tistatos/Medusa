cmake_minimum_required(VERSION 2.8.7)

set(PROJECT_NAME medusa)
project(${PROJECT_NAME} C CXX)
include(ExternalProject)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(Threads REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

set(PROJECT_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include)
set(PROJECT_LIBS_DIR ${PROJECT_SOURCE_DIR}/libs)
set(PROJECT_CXX_DIR ${PROJECT_SOURCE_DIR}/src)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_SOURCE_DIR}/cmake/Modules/)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "You must create a build folder and call cmake .. from there!, Also remove CMakeCache.txt before trying to cmake again")
endif()

###############
# PCL Settings
###############

set(PCL_DIR "/path/to/PCLConfig.cmake")
find_package(PCL 1.7 REQUIRED )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${EIGEN_INCLUDE_DIRS})


##################
# MongoDB Settings
##################
find_package(bsoncxx REQUIRED)
find_package(mongocxx REQUIRED)
include_directories(${BSONCXX_INCLUDE_DIRS})
include_directories(${MONGOCXX_INCLUDE_DIRS})
link_directories(${MONGOCXX_LIBRARY_DIRS})
link_directories(${BSONCXX_LIBRARY_DIRS})


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

if(APPLE)
  set(FREENECT_SYNC_LIB libfreenect_sync.dylib)
ELSE()
  set(FREENECT_SYNC_LIB libfreenect_sync.so)
ENDIF()

##################
# libwebsockets
##################
if(APPLE)
find_package(libwebsockets REQUIRED )
include_directories(${libwebsockets_INCLUDE_DIRS})
link_directories(${LIBWEBSOCKETS_LIBRARY_DIRS})
ELSE()
   FIND_PATH(LIBWEBSOCKETS_INCLUDES
    libwebsockets.h
    PATHS ${LIBWEBSOCKETS_INCLUDE_PATH} /usr/include)

  FIND_LIBRARY(LIBWEBSOCKETS_LIBRARIES
    NAMES libwebsockets websockets
    PATHS ${LIBWEBSOCKETS_LIBRARIES} /usr/lib64 /usr/lib)
ENDIF()

##################
# BOOST
##################

set(Boost_USE_STATIC_LIBS        OFF) # only find static libs
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)
set(Boost_DEBUG 1)

FIND_PACKAGE( Boost 1.46 REQUIRED COMPONENTS regex)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIRS} )

##################
# targets
##################
include_directories(
        ${PROJECT_INCLUDE_DIR}
)

message(${Boost_LIBRARIES})
set(ALL_LIBS ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${FREENECT_LIBRARIES} ${FREENECT_SYNC_LIB} ${LIBWEBSOCKETS_LIBRARIES} ${BSONCXX_LIBRARIES} ${MONGOCXX_LIBRARIES} ${Boost_LIBRARIES} mongoclient bson png)

add_library(rendermesh src/renderMesh.cpp)
target_link_libraries(rendermesh ${ALL_LIBS})

add_library(texture src/texture.cpp)
target_link_libraries(texture ${PCL_LIBRARIES})

add_library(kinectmanager src/KinectManager.cpp src/Kinect.cpp)
target_link_libraries(kinectmanager ${FREENECT_LIBRARIES})

add_executable(medusa src/main.cpp src/Medusa.cpp src/websocket.cpp)
target_link_libraries(medusa texture kinectmanager rendermesh ${ALL_LIBS})

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${FREENECT_LIBRARIES} ${FREENECT_SYNC_LIB} ${OpenCV_LIBS})

add_executable(write_xml_file src/write_xml_file.cpp)
target_link_libraries(write_xml_file ${OpenCV_LIBS})

##################
# testing
##################
if(${BUILD_TESTS})
add_subdirectory(
  ${PROJECT_SOURCE_DIR}/gtest
)

enable_testing()

set(PROJECT_TEST_NAME ${PROJECT_NAME}_test)
include_directories(${GTEST_INCLUDE_DIRS} ${COMMON_INCLUDES})

file(GLOB TEST_SRC_FILES ${PROJECT_SOURCE_DIR}/test/*.cpp)

add_executable(${PROJECT_TEST_NAME} ${TEST_SRC_FILES} )
add_dependencies(${PROJECT_TEST_NAME} googletest)

target_link_libraries(${PROJECT_TEST_NAME}
    ${ALL_LIBS}
    ${GTEST_LIBS_DIR}/ReleaseLibs/libgtest.a
    ${GTEST_LIBS_DIR}/ReleaseLibs/libgtest_main.a
    )
target_link_libraries(${PROJECT_TEST_NAME} ${CMAKE_THREAD_LIBS_INIT})
add_test(test ${PROJECT_TEST_NAME})
endif()
