find_package(Boost)
find_package(Threads REQUIRED)
find_package(OpenSSL REQUIRED)

include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz)
FetchContent_MakeAvailable(json)

include_directories(${Boost_INCLUDE_DIRS})

add_library(gbemu_lib
    src/cpu.cpp
    src/executor.cpp
    src/instructions.cpp
    src/mmu.cpp
    src/ram.cpp
)

target_include_directories(gbemu_lib PUBLIC include)
target_link_libraries(gbemu_lib PUBLIC nlohmann_json::nlohmann_json)