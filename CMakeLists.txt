cmake_minimum_required(VERSION 2.8.3)
set(project "scs")
project(${project})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

file(GLOB hdrs src/*.h)
file(GLOB srs src/*.cpp)

add_library(${project} STATIC ${hdrs} ${srs})
add_executable(${project}_test src/test.cpp)
target_link_libraries(${project}_test ${project})
