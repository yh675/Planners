cmake_minimum_required(VERSION 3.0.0)
project(Planners VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

include(CTest)
enable_testing()

add_executable(Planners main.cpp)
target_link_libraries(Planners ${OpenCV_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
SET(CMAKE_CXX_FLAGS "-std=c++11")
include(CPack)
