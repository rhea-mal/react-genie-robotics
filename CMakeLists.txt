cmake_minimum_required (VERSION 3.0)
project (CENTER_DEMO)
if (${CMAKE_SYSTEM_NAME} MATCHES Linux)
	SET(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -O3")
elseif (${CMAKE_SYSTEM_NAME} MATCHES Darwin)
	SET(CMAKE_CXX_FLAGS "-std=c++17 -I/usr/include -I/usr/local/include -I/opt/homebrew/include -w -O3")
endif()
SET(CMAKE_BUILD_TYPE "Debug")  
# SET(CMAKE_BUILD_TYPE "Release")  

# set common dependencies
# - eigen3
set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_INCLUDE_DIR})
if(NOT EIGEN3_INCLUDE_DIR)
	find_package(Eigen3 QUIET)
	# TODO: Find automatic way to find for Ubuntu 14.04
	if(NOT EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)
	endif()
endif()
include_directories(${EIGEN3_INCLUDE_DIR})

# include Chai3d 
# -- this is required by Chai3d to pick the correct path for OpenGL headers
FIND_PACKAGE(CHAI3D REQUIRED)
ADD_DEFINITIONS(${CHAI3D_DEFINITIONS})
if (CMAKE_SYSTEM_NAME MATCHES Darwin) 
  ADD_DEFINITIONS(-D_MACOSX -DMACOSX)
elseif (CMAKE_SYSTEM_NAME MATCHES Linux)
  ADD_DEFINITIONS(-D_LINUX -DLINUX)
endif () #NOTE: Windows is currently unsupported
LINK_DIRECTORIES(${CHAI3D_LIBRARY_DIRS})
INCLUDE_DIRECTORIES(${CHAI3D_INCLUDE_DIRS})

# - SAI2-SIMULATION
find_package(SAI2-SIMULATION REQUIRED)
include_directories(${SAI2-SIMULATION_INCLUDE_DIRS})
add_definitions(${SAI2-SIMULATION_DEFINITIONS})

# - SAI2-GRAPHICS
FIND_PACKAGE(SAI2-GRAPHICS REQUIRED)
include_directories(${SAI2-GRAPHICS_INCLUDE_DIRS})
add_definitions(${SAI2-GRAPHICS_DEFINITIONS})

# - SAI2-COMMON
FIND_PACKAGE(SAI2-COMMON REQUIRED)
include_directories(${SAI2-COMMON_INCLUDE_DIRS})

# - SAI2-URDF
FIND_PACKAGE(SAI2-URDF REQUIRED)
include_directories(${SAI2-URDF_INCLUDE_DIRS})

# - SAI2-MODEL
find_package(SAI2-MODEL REQUIRED)
include_directories(${SAI2-MODEL_INCLUDE_DIRS})
add_definitions(${SAI2-MODEL_DEFINITIONS})

# - SAI2-PRIMITIVES
find_package(SAI2-PRIMITIVES REQUIRED)
include_directories(${SAI2-PRIMITIVES_INCLUDE_DIRS})
add_definitions(${SAI2-PRIMITIVES_DEFINITIONS})

# - SAI2-INTERFACES
find_package(SAI2-INTERFACES REQUIRED)
include_directories(${SAI2-INTERFACES_INCLUDE_DIRS})
add_definitions(${SAI2-INTERFACES_DEFINITIONS})

# - hiredis
find_library(HIREDIS_LIBRARY hiredis)

# - library
add_library(center-demo STATIC ${PROJECT_SOURCE_DIR}/include/Human.cpp ${PROJECT_SOURCE_DIR}/include/Human2D.cpp)


# define libraries for linking 
set(CENTER_DEMO_COMMON_LIBRARIES
    ${SAI2-INTERFACES_LIBRARIES}
    ${SAI2-MODEL_LIBRARIES}
    ${SAI2-GRAPHICS_LIBRARIES}
    ${SAI2-SIMULATION_LIBRARIES}
    ${SAI2-COMMON_LIBRARIES}
    ${SAI2-URDF_LIBRARIES}
    ${CHAI3D_LIBRARIES}
    ${HIREDIS_LIBRARY}
    ${SAI2-PRIMITIVES_LIBRARIES}
    ${GLFW_LIBRARY}
    center-demo
	)

set (CENTER_DEMO_BINARY_DIR ${PROJECT_SOURCE_DIR}/bin)

add_subdirectory(optitrack)