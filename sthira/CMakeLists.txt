cmake_minimum_required(VERSION 3.14)
project(sthira)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(sthira STATIC
    src/sthira.cpp
)
target_include_directories(sthira PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(sthira PUBLIC cxx_std_17)

