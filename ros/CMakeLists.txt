cmake_minimum_required(VERSION 2.8.12)
set(project "feetech_sdk")
project(${project})

find_package(catkin REQUIRED COMPONENTS roscpp)

catkin_package(
        INCLUDE_DIRS ../include
        CATKIN_DEPENDS roscpp
        LIBRARIES ${project}
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    add_compile_options(/std:c++14)
else()
    add_compile_options(-std=c++14)
endif()

include_directories(
        ../include
        ${catkin_INCLUDE_DIRS}
)
file(GLOB srcs ../src/*.cpp)

add_library(${project} ${hdrs} ${srcs})

install(TARGETS ${project}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ../include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
