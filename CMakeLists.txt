cmake_minimum_required(VERSION 3.26)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(BUILD_DEPS ON)
set (CMAKE_BUILD_TYPE "Release" CACHE
      STRING "Choose the type of build.")
set(CMAKE_CXX_STANDARD 23)

project(gbbcpp_emu)

if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    execute_process(
        COMMAND ${CMAKE_CXX_COMPILER} -print-file-name=libstdc++.so.6
        OUTPUT_VARIABLE GNU_LIBSTDCXX
        OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    get_filename_component(GNU_LIBSTDCXX_DIR "${GNU_LIBSTDCXX}" DIRECTORY)
    if (GNU_LIBSTDCXX_DIR AND NOT GNU_LIBSTDCXX_DIR MATCHES "^/usr/lib|^/lib")
        list(APPEND CMAKE_BUILD_RPATH "${GNU_LIBSTDCXX_DIR}")
        list(APPEND CMAKE_INSTALL_RPATH "${GNU_LIBSTDCXX_DIR}")
    endif()
endif()

find_package(Boost REQUIRED CONFIG)
find_package(portable-file-dialogs REQUIRED CONFIG)
find_package(SDL2 REQUIRED CONFIG)
find_package(SDL2_ttf REQUIRED CONFIG)
find_package(Threads REQUIRED)

add_subdirectory(app)
add_subdirectory(lib)
