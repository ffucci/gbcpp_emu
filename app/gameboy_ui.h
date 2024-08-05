#pragma once

#include <SDL.h>
#include <SDL_rect.h>
#include <SDL_render.h>
#include <SDL_video.h>
#include <SDL2/SDL_ttf.h>

#include <sys/types.h>
#include <cstdint>
#include "cpu/cpu.h"
#include "ppu/ppu_context.h"

namespace gameboy::ui {
struct Emulation
{
    Emulation(cpu::CPU& cpu) : gameboy_cpu(cpu)
    {
    }

    cpu::CPU& gameboy_cpu;
    int width{1024};
    int height{768};

    bool stop{false};
};

class GameboyUI
{
   public:
    GameboyUI() = delete;

    GameboyUI(Emulation& emulation) : emulation_(emulation)
    {
        SDL_Init(SDL_INIT_VIDEO);
        TTF_Init();

        window_ = SDL_CreateWindow(
            "gbemu_cpp", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, emulation_.width, emulation_.height,
            SDL_WINDOW_OPENGL);

        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
        screen_ = SDL_CreateRGBSurface(
            0, emulation_.width, emulation.height, 32, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);

        SDL_CreateWindowAndRenderer(
            16 * 8 * GameboyUI::SCALE, 32 * 8 * GameboyUI::SCALE, 0, &debug_window_, &debug_renderer_);

        texture_ = SDL_CreateTexture(
            renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, emulation_.width, emulation_.height);

        debug_texture_ = SDL_CreateTexture(
            debug_renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
            (16 * 8 * GameboyUI::SCALE) + 16 * SCALE, (32 * 8 * GameboyUI::SCALE) + (64 * SCALE));

        debug_screen_ = SDL_CreateRGBSurface(
            0, (16 * 8 * SCALE) + (16 * SCALE), (32 * 8 * SCALE) + (64 * SCALE), 32, 0x00FF0000, 0x0000FF00, 0x000000FF,
            0xFF000000);

        int x, y;

        SDL_GetWindowPosition(window_, &x, &y);
        SDL_SetWindowPosition(debug_window_, x + emulation_.width + 10, y);
    }

    void ui_update()
    {
        const auto& ppu_context = emulation_.gameboy_cpu.memory().ppu().context();
        if (prev_frame_ != ppu_context.current_frame) {
            update_main();
            update_debug_window();
        }
        prev_frame_ = ppu_context.current_frame;
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

    void update_main()
    {
        SDL_Rect rc;
        rc.x = rc.y = 0;
        rc.w = rc.h = 2048;

        auto& video_buffer = emulation_.gameboy_cpu.memory().ppu().context().video_buffer;

        for (int line_num = 0; line_num < ppu::PPUContext::YRES; line_num++) {
            for (int x = 0; x < ppu::PPUContext::XRES; x++) {
                rc.x = x * SCALE;
                rc.y = line_num * SCALE;
                rc.w = SCALE;
                rc.h = SCALE;

                SDL_FillRect(screen_, &rc, video_buffer[x + (line_num * ppu::PPUContext::XRES)]);
            }
        }

        SDL_UpdateTexture(texture_, NULL, screen_->pixels, screen_->pitch);
        SDL_RenderClear(renderer_);
        SDL_RenderCopy(renderer_, texture_, NULL, NULL);
        SDL_RenderPresent(renderer_);
    }

    void update_debug_window()
    {
        int x_draw = 0;
        int y_draw = 0;
        int tile_num = 0;
        SDL_Rect rect;
        rect.x = 0;
        rect.y = 0;
        rect.w = debug_screen_->w;
        rect.h = debug_screen_->h;

        SDL_FillRect(debug_screen_, &rect, GRAY_COLOR);
        uint16_t addr = 0x8000;

        // 384 tiles, 24 x 16
        constexpr uint16_t TILES_Y{24};
        constexpr uint16_t TILES_X{16};

        for (int y = 0; y < TILES_Y; y++) {
            for (int x = 0; x < TILES_X; x++) {
                display_tile(debug_screen_, addr, tile_num, x_draw + (x * SCALE), y_draw + (y * SCALE));
                x_draw += (8 * SCALE);
                tile_num++;
            }

            y_draw += (8 * SCALE);
            x_draw = 0;
        }

        SDL_UpdateTexture(debug_texture_, NULL, debug_screen_->pixels, debug_screen_->pitch);
        SDL_RenderClear(debug_renderer_);
        SDL_RenderCopy(debug_renderer_, debug_texture_, NULL, NULL);
        SDL_RenderPresent(debug_renderer_);
    }

    void display_tile(SDL_Surface* surface, uint16_t start_location, uint16_t tile_num, int x, int y)
    {
        SDL_Rect rc;
        auto& memory = emulation_.gameboy_cpu.memory();
        for (int tile_y = 0; tile_y < 16; tile_y += 2) {
            const uint16_t pos = start_location + (tile_num * 16) + tile_y;
            auto b1 = memory.read(pos);      // 2 bytes per line
            auto b2 = memory.read(pos + 1);  // 2 bytes per line

            for (int bit = 7; bit >= 0; bit--) {
                uint8_t hi = !!(b1 & (1 << bit)) << 1;
                uint8_t lo = !!(b2 & (1 << bit));

                uint8_t color = hi | lo;

                rc.x = x + ((7 - bit) * SCALE);
                rc.y = y + (tile_y / 2 * SCALE);
                rc.w = SCALE;
                rc.h = SCALE;

                // Drawing the pixel
                SDL_FillRect(surface, &rc, TILE_COLORS[color]);
            }
        }
    }

    SDL_Window* window_;
    SDL_Window* debug_window_;

    SDL_Surface* screen_;
    SDL_Surface* debug_screen_;

    SDL_Renderer* renderer_;
    SDL_Renderer* debug_renderer_;

    SDL_Texture* texture_;
    SDL_Texture* debug_texture_;

    Emulation& emulation_;

    uint32_t prev_frame_{0};

    static constexpr uint32_t SCALE{4};
    static constexpr uint32_t GRAY_COLOR{0xFF111111};

    static constexpr uint32_t TILE_COLORS[4] = {0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000};
};
}  // namespace gameboy::ui
