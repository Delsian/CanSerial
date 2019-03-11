cmake_minimum_required(VERSION 3.3)
project(canmod)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES canserial.c
	portnumber.c portnumber.h
	cansock.c cansock.h)

include_directories(
        /usr/local/include
)

link_directories(
        /usr/local/lib
)

add_executable(canserial ${SOURCE_FILES})

target_link_libraries(canserial
		pthread
		util
        )
        