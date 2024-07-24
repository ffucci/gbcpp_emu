find_package(Boost)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

add_library(gbemu_lib
    src/cpu.cpp
    src/instruction_handlers.cpp
    src/instructions.cpp
    src/interrupt.cpp
    src/mmu.cpp
    src/ram.cpp
    src/timer.cpp
    src/device.cpp
)

target_include_directories(gbemu_lib PUBLIC include)

add_subdirectory(tests)