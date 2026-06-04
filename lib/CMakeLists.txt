add_library(gbemu_lib
    src/cpu.cpp
    src/gamepad.cpp
    src/instruction_handlers.cpp
    src/instructions.cpp
    src/interrupt.cpp
    src/mbc1.cpp
    src/mbc3.cpp
    src/mbc5.cpp
    src/mmu.cpp
    src/ram.cpp
    src/timer.cpp
    src/device.cpp
    src/ppu.cpp
    src/dma.cpp
    src/emulator_session.cpp
)

target_include_directories(gbemu_lib PUBLIC include)
target_link_libraries(
    gbemu_lib
    PUBLIC
        Boost::boost
        SDL2::SDL2
        Threads::Threads
)

add_subdirectory(tests)
