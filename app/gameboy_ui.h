#pragma once

#include <SDL.h>
#include <SDL_render.h>
#include <sys/types.h>
#include <boost/asio/io_context.hpp>
#include <cstdint>
#include "cpu/cpu.h"

namespace gameboy::ui {
struct Emulation
{
    Emulation(cpu::CPU& cpu) : gameboy_cpu(cpu)
    {
    }

    cpu::CPU& gameboy_cpu;
    int width{400};
    int height{400};

    bool stop{false};
};

class GameboyUI
{
   public:
    GameboyUI() = delete;

    GameboyUI(Emulation& emulation, boost::asio::io_context& ioc) : emulation_(emulation), ioc_(ioc)
    {
        SDL_Init(SDL_INIT_VIDEO);

        window_ = SDL_CreateWindow(
            "gbemu_cpp", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, emulation_.width, emulation_.height,
            SDL_WINDOW_OPENGL);

        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

        texture_ = SDL_CreateTexture(
            renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, emulation_.width, emulation_.height);
    }

    void wait_for_events()
    {
        SDL_Event e;
        while (SDL_PollEvent(&e) > 0) {
            // TODO SDL_UpdateWindowSurface(sdlWindow);
            // TODO SDL_UpdateWindowSurface(sdlTraceWindow);
            // TODO SDL_UpdateWindowSurface(sdlDebugWindow);

            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE) {
                emulation_.stop = true;
            }
        }
    }

    ~GameboyUI()
    {
        SDL_DestroyTexture(texture_);
        SDL_DestroyRenderer(renderer_);
        SDL_DestroyWindow(window_);
        SDL_Quit();
    }

   private:
    void delay(uint32_t ms)
    {
        SDL_Delay(ms);
    }

    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Texture* texture_;

    Emulation& emulation_;
    boost::asio::io_context& ioc_;
};
}  // namespace gameboy::ui
