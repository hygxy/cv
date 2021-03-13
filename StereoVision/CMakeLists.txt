cmake_minimum_required(VERSION 2.8)
set(PROG_NAME CV)
project (${PROG_NAME} LANGUAGES CXX )

#SET(CMAKE_PREFIX_PATH /path_to_opencv)
#SET(OpenCV_DIR /path_to_opencv)
SET(CMAKE_BUILD_TYPE RELWITHDEBINFO)



# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
SET(LIBS ${LIBS} ${OpenCV_LIBS})

#openmp
find_package(OpenMP)
if (OPENMP_FOUND)
	include_directories(${OPENMP_INCLUDE_DIRS})
	SET(LIBS ${LIBS} ${OPENMP_LIBRARIES})
    	set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
	set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

if(UNIX)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-long-long -pedantic")
endif(UNIX)
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

############# SOURCE ###############

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/lib)

file(GLOB SRC src/lib/*.cpp)
file(GLOB HEADER src/lib/*.h)
add_library (computerVision STATIC ${SRC} ${HEADER})
target_link_libraries( computerVision ${LIBS} )


file(GLOB SRC src/stereoCalibration/*.cpp)
file(GLOB HEADER src/stereoCalibration/*.h)
add_executable (stereoCalibration ${SRC} ${HEADER})
target_link_libraries( stereoCalibration computerVision ${LIBS} )

file(GLOB SRC src/stereoVision/*.cpp)
file(GLOB HEADER src/stereoVision/*.h)
add_executable (stereoVision ${SRC} ${HEADER})
target_link_libraries( stereoVision computerVision ${LIBS} )

file(GLOB SRC src/stereoDataset/*.cpp)
file(GLOB HEADER src/stereoDataset/*.h)
add_executable (stereoDataset ${SRC} ${HEADER})
target_link_libraries( stereoDataset computerVision ${LIBS} )


#############  output dir #############

#copies executable to top level source tree
set_target_properties( stereoCalibration
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
        
set_target_properties( stereoVision
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
        
set_target_properties( stereoDataset
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
        
