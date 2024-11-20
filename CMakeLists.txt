cmake_minimum_required(VERSION 3.14)

project(arches-v2 LANGUAGES CXX)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(external)
add_subdirectory("src")
