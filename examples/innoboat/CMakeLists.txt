cmake_minimum_required(VERSION 3.10.2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(innoboat)

add_executable(innoboat
    innoboat.cpp
)

find_package(MAVSDK REQUIRED)

target_link_libraries(innoboat
    MAVSDK::mavsdk
)

if(NOT MSVC)
    add_compile_options(innoboat PRIVATE -Wall -Wextra)
else()
    add_compile_options(innoboat PRIVATE -WX -W2)
endif()
