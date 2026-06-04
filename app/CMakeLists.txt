add_executable(emulator emulator.cpp emulator_session.cpp)
target_include_directories(emulator PUBLIC include)
target_link_libraries(
    emulator
    PRIVATE
        gbemu_lib
        portable-file-dialogs::portable-file-dialogs
        SDL2::SDL2
        sdl_ttf::sdl_ttf
        Threads::Threads
)
