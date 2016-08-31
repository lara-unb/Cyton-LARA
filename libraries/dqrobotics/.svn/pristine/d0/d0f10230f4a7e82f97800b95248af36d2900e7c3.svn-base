cmake_minimum_required (VERSION 2.6) 
project (DQ_ROBOTICS) 

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
find_package(GMATRIX REQUIRED)
include_directories(${GMATRIX_INCLUDE_DIRS})
set(LIBS ${LIBS} ${GMATRIX_LIBRARIES})

add_subdirectory (DQ)
add_subdirectory (examples) 
add_subdirectory (kinematics)
