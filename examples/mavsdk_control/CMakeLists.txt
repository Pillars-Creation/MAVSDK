cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(mavsdk_control)

add_executable(mavsdk_control
    mavsdk_control.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(mavsdk_control
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(mavsdk_control PRIVATE -Wall -Wextra)
else()
    add_compile_options(mavsdk_control PRIVATE -W2)
endif()
