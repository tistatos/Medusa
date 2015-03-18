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
  message(FATAL_ERROR "You must create a build folder and call cmake .. from there!, Also remove CMakeCache.txt before trying to cmake again")
endif()

###############
# PCL Settings
###############

set(PCL_DIR "/path/to/PCLConfig.cmake")
find_package(PCL 1.7 COMPONENTS )
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
message(${PCL_LIBRARY_DIRS})
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

if(APPLE)
  set(FREENECT_SYNC_LIB libfreenect_sync.dylib)
ELSE()
  set(FREENECT_SYNC_LIB libfreenect_sync.so)
ENDIF()

##################
# targets
##################
include_directories(
        ${PROJECT_INCLUDE_DIR}
)


set(ALL_LIBS ${PCL_LIBRARIES} ${OpenCV_LIBS} ${FREENECT_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT} ${FREENECT_LIBRARIES} ${FREENECT_SYNC_LIB})

add_library(kinectmanager src/KinectManager.cpp src/Kinect.cpp)
target_link_libraries(kinectmanager ${FREENECT_LIBRARIES})

add_executable(medusa src/main.cpp)
target_link_libraries(medusa kinectmanager ${ALL_LIBS})

add_executable(kinect_rgb src/kinect.cpp)
target_link_libraries(kinect_rgb ${ALL_LIBS})

add_executable(calibration src/calibration.cpp)
target_link_libraries(calibration ${FREENECT_LIBRARIES} ${FREENECT_SYNC_LIB} ${OpenCV_LIBS})

add_executable(write_xml_file src/write_xml_file.cpp)
target_link_libraries(write_xml_file ${OpenCV_LIBS})

add_executable(mainRender src/mainRender.cpp src/kinect_depth.cpp src/renderMesh.cpp)
target_link_libraries(mainRender ${ALL_LIBS})

add_executable (openni_grabber src/openni_grabber.cpp)
target_link_libraries (openni_grabber ${PCL_LIBRARIES})

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
