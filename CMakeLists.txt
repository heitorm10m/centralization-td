cmake_minimum_required(VERSION 3.21)

project(centraltd VERSION 0.1.0 LANGUAGES CXX)

option(CENTRALTD_BUILD_TESTS "Build C++ smoke tests" ON)
option(CENTRALTD_BUILD_PYTHON "Build Python bindings" ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

include(CTest)

add_subdirectory(cpp)
