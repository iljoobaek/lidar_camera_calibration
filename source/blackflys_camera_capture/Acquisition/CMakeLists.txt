cmake_minimum_required( VERSION 3.6 )

# Require C++11
set( CMAKE_CXX_STANDARD 11 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )
set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/") 
MESSAGE(STATUS "CMAKE_MODULE_PATH = ${CMAKE_MODULE_PATH}" )

# Create Project
project( blackflys_capture )
set(EXECUTABLE blackflys_capture)

add_executable( ${EXECUTABLE} date/tz.h Acquisition.cpp )

# Find Package Thread
# find_package( Threads REQUIRED )


# Find Package Spinnaker
find_package( Spinnaker REQUIRED )
MESSAGE(STATUS "SPINNAKER_INCLUDE_DIRS = ${SPINNAKER_INCLUDE_DIRS}" )
MESSAGE(STATUS "SPINNAKER_LIB = ${SPINNAKER_LIB} " )

# Find Package OpenCV
find_package( OpenCV REQUIRED )
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}" )
MESSAGE(STATUS "OpenCV_LIB_DIRS = ${OpenCV_LIB_DIRS} " )
MESSAGE(STATUS "OpenCV_LIBS = ${OpenCV_LIBS} " )

# Required Viz Module
if( OpenCV_FOUND )
  if( NOT "opencv_viz" IN_LIST OpenCV_LIBS )
    message( FATAL_ERROR "not found opencv_viz module." )
  endif()
endif()

# Set Properties
if( OpenCV_FOUND )
  # Additional Include Directories
  include_directories( ${OpenCV_INCLUDE_DIRS} )
  include_directories( ${SPINNAKER_INCLUDE_DIRS} )

  # Additional Library Directories
  link_directories( ${OpenCV_LIB_DIR} )

  # Additional Dependencies
  target_link_libraries( ${EXECUTABLE} ${SPINNAKER_LIB} )
  target_link_libraries( ${EXECUTABLE} ${OpenCV_LIBS} )
endif()

target_link_libraries( ${EXECUTABLE} stdc++fs )
