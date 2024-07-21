#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>

#include "cartridge/cartridge.h"
#include "gameboy_ui.h"
#include "utils/logger.h"
#include "utils/utils.h"

#include <SDL.h>

#include <boost/asio.hpp>

#include "cpu/cpu.h"

void close_handler(const boost::system::error_code& error, int signal_number)
{
    using namespace gameboy::logger;
    auto& logger = Logger::instance();
    logger.log("<-------- Closing GB-EMU -------->");
}

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

    boost::asio::io_context ioc;

    gameboy::cartridge::Cartridge cartridge(filename);
    gameboy::cpu::CPU cpu(gameboy::cpu::get_initial_state(), cartridge);

    gameboy::ui::Emulation emulation(cpu);
    gameboy::ui::GameboyUI gameboy(emulation, ioc);

    // boost::asio::signal_set signals(ioc, SIGINT);
    // signals.async_wait([&ioc](auto error, auto signal_nr) {
    //     ioc.stop();
    //     close_handler(error, signal_nr);
    // });

    std::jthread cpu_thread([&cpu](std::stop_token token) { cpu.run(token); });

    while (!emulation.stop) {
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        gameboy.wait_for_events();
    }

    cpu_thread.request_stop();
    return 0;
}