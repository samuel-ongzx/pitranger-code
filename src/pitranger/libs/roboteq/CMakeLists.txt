cmake_minimum_required(VERSION 3.1)

project(roboteq)
# set(CMAKE_CXX_STANDARD 14)
add_compile_options(-std=c++11)

add_library(${PROJECT_NAME}
	src/Constants.h
	src/ErrorCodes.h
	src/RoboteqDevice.cpp
	src/RoboteqDevice.h
  	src/Roboteq.h
)
target_include_directories(${PROJECT_NAME} PUBLIC src/)
