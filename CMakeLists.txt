cmake_minimum_required(VERSION 3.26)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(BUILD_DEPS ON)
set (CMAKE_BUILD_TYPE "Release" CACHE
      STRING "Choose the type of build.")
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_COMPILER clang++)

project(gbbcpp_emu)

add_subdirectory(app)
add_subdirectory(lib)