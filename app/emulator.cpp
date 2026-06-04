#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>

#include "../lib/include/utils/emulator_session.h"
#include "gameboy_ui.h"
#include "utils/logger.h"
#include "utils/utils.h"

#include <SDL.h>

int main(int argc, char** argv)
{
    using namespace gameboy::logger;
    auto& logger = Logger::instance();
    logger.log("... Starting GB-EMU .... ");
    logger.log("< Written by Francesco Fucci >");

    gameboy::app::EmulatorSession session;
    if (argc >= 2) {
        session.load_rom(argv[1]);
    }

    gameboy::ui::Emulation emulation(session);
    gameboy::ui::GameboyUI gameboy(emulation);

    while (!emulation.stop) {
        std::this_thread::sleep_for(std::chrono::microseconds(1000));

        gameboy.wait_for_events();
        gameboy.ui_update();
    }

    session.stop();
    return 0;
}
