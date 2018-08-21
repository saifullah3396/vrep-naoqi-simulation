cmake_minimum_required(VERSION 2.8)

# Project name
project(vrep-naoqi-simulation)

# Additional cmake configuration path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Add the build parameters
add_definitions(
  -DNAOQI_SIM_SDK="$ENV{PATH_TO_SIM_DIR}" # Naoqi simulator sdk path
  -DNON_MATLAB_PARSING=1 # Vrep configuration parameter
  -DMAX_EXT_API_CONNECTIONS=255 # Vrep max connections parameter
  -std=gnu++0x) # allow gnu++0x syntax

# Find naoqi modules using cmake/FindNAOqiSIM.cmake
find_package(NAOqiSIM REQUIRED COMPONENTS 
	alnaosim 
	alsimutils
	alrobotmodel
	alproxies
	almemoryfastaccess
	alcommon	  
	qi
	qitype
)

# Check if naoqi modules are found
# Adapted from 
if(NAOqiSIM_FOUND)
	message(STATUS "NAOqi Simulator SDK found!")
endif()

# Set BOOST_LIBS variable to boost libraries from sdk thus avoiding 
# conflict with system libs
set(BOOST_LIBS
  $ENV{PATH_TO_SIM_DIR}/lib/libboost_system.so)

# Set BOOST_INCLUDE_DIRS variable to boost includes from sdk thus avoiding 
# conflict with system dirs
set(BOOST_INCLUDE_DIRS
  $ENV{PATH_TO_SIM_DIR}/include)

# Set the root folder dir
set(MAIN $ENV{PATH_TO_TEAM_NUST_DIR})

# Set the source files for the simulator interface
FILE(GLOB SOURCE_MAIN 
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
)

# Set the include dirs for the simulator interface
include_directories(
  include 
  ${NAOqiSIM_INCLUDE_DIRS} 
  ${BOOST_INCLUDE_DIRS})

# Build the simulator interface
add_executable(vrep-naoqi-simulation ${SOURCE_MAIN})

# Linking libraries
target_link_libraries(
  vrep-naoqi-simulation 
  ${NAOqiSIM_LIBRARIES} 
  ${BOOST_LIBS} 
  pthread)
