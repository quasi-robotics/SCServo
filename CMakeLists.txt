cmake_minimum_required(VERSION 2.8.4)
cmake_policy(SET CMP0054 NEW)
set(project "scs")
project(${project})

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++14")
else()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
endif()

file(GLOB srcs src/*.cpp)

add_library(${project} STATIC ${srcs})
add_executable(${project}_test src/test.cpp)
target_link_libraries(${project}_test ${project})
