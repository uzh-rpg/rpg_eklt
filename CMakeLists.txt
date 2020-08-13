cmake_minimum_required(VERSION 2.8.3)
project(synchronizer)

find_package(catkin_simple REQUIRED )
find_package(OpenCV)

catkin_simple()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

set(HEADERS
    include/synchronizer.h
    include/synchronizer_online.h
)

set(SOURCES
    src/synchronizer.cpp
    src/synchronizer_online.cpp
)

cs_add_library(libsync ${SOURCES} ${HEADERS})

# make executable
cs_add_executable(synchronizer src/synchronizer_node.cpp)
target_link_libraries(synchronizer libsync ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

cs_install()
cs_export()