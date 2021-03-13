cmake_minimum_required(VERSION 2.8)
set(PROG_NAME CV)
project (${PROG_NAME} LANGUAGES CXX )

#SET(CMAKE_PREFIX_PATH /path_to_opencv)
#SET(OpenCV_DIR /path_to_opencv)
SET(CMAKE_BUILD_TYPE DEBUG)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# OpenCV
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )
SET(LIBS ${LIBS} ${OpenCV_LIBS})



if(UNIX)
        SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wno-long-long -pedantic")
endif(UNIX)
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

############# SOURCE ###############

file(GLOB SRC src/*.cpp)
message(STATUS "Files: ${SRC}")

add_executable (${PROG_NAME} ${SRC})
target_link_libraries( ${PROG_NAME} ${LIBS} )


#############  output dir #############

#copies executable to top level source tree
set_target_properties( ${PROG_NAME}
        PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}")
