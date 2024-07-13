find_package(SDL2)
find_package(Boost REQUIRED)
find_package(Threads REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

if (SDL2_FOUND)
    add_executable(emulator emulator.cpp)
    target_include_directories(emulator PUBLIC include)

    include_directories(SYSTEM ${SDL2_INCLUDE_DIRS})
    target_link_libraries(emulator ${SDL2_LIBRARIES})
endif()

