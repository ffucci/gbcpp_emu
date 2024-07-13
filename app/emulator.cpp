#include <chrono>
#include <cstddef>
#include <cstring>
#include <thread>
#include "cartridge/cartridge_header.h"
#include "utils/logger.h"
#include "utils/utils.h"

#include <SDL.h>

#include <boost/asio.hpp>

void close_handler(const boost::system::error_code& error, int signal_number)
{
    auto& logger = Logger::instance();
    logger.log("<-------- Closing GB-EMU -------->");
}

int main()
{
    auto& logger = Logger::instance();
    logger.log("... Starting GB-EMU .... ");
    logger.log("< Written by Francesco Fucci >");
    auto rom_bytes = read_rom("./games/Pokemon_ES.gb");
    auto rom_header = from_rom(rom_bytes);

    SDL_Init(SDL_INIT_VIDEO);

    const auto width = 400;
    const auto height = 400;
    auto window = SDL_CreateWindow(
        "gbemu_cpp", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_OPENGL);

    auto renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    auto gb_screen_texture =
        SDL_CreateTexture(renderer, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, width, height);

    boost::asio::io_context ioc;
    boost::asio::signal_set signals(ioc, SIGINT);
    signals.async_wait([&ioc](auto error, auto signal_nr) {
        ioc.stop();
        close_handler(error, signal_nr);
    });

    ioc.run();

    SDL_DestroyTexture(gb_screen_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}