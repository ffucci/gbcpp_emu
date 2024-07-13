#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>
#include "cartridge/cartridge_header.h"
#include "utils/logger.h"
#include "utils/utils.h"

int main()
{
    auto& logger = Logger::instance();
    logger.log("... Starting GB-EMU .... ");
    logger.log("< Written by Francesco Fucci >");
    auto rom_bytes = read_rom("./games/Pokemon_ES.gb");
    auto rom_header = from_rom(rom_bytes);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    return 0;
}