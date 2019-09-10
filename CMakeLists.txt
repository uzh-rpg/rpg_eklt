cmake_minimum_required(VERSION 2.8.3)
project(eklt)

find_package(catkin_simple REQUIRED)
find_package(OpenCV REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -fopenmp")
SET(CMAKE_BUILD_TYPE Release)

set(HEADERS
    include/error.h
    include/optimizer.h
    include/patch.h
    include/tracker.h
    include/viewer.h
)

set(SOURCES
    src/optimizer.cpp
    src/tracker.cpp
    src/viewer.cpp
)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

# make executable
cs_add_executable(eklt_node src/eklt_node.cpp)
target_link_libraries(eklt_node ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})

cs_install()
cs_export()