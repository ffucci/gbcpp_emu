#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>

#include "cartridge/cartridge.h"
#include "gameboy_ui.h"
#include "utils/logger.h"
#include "utils/utils.h"

#include <SDL.h>

#include "cpu/cpu.h"

int main(int argc, char** argv)
{
    if (argc < 2) {
        printf("Usage: emu <rom_file>\n");
        return -1;
    }

    std::string filename(argv[1]);

    using namespace gameboy::logger;
    auto& logger = Logger::instance();
    logger.log("... Starting GB-EMU .... ");
    logger.log("< Written by Francesco Fucci >");

    gameboy::cartridge::Cartridge cartridge(filename);
    gameboy::cpu::CPU cpu(gameboy::cpu::get_initial_state(), cartridge);

    gameboy::ui::Emulation emulation(cpu);
    gameboy::ui::GameboyUI gameboy(emulation);
    std::jthread cpu_thread([&cpu](std::stop_token token) { cpu.run(token); });

    while (!emulation.stop) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));

        gameboy.wait_for_events();
        gameboy.ui_update();
    }

    cpu_thread.request_stop();
    return 0;
}