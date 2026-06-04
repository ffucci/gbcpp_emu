#pragma once

#include <SDL.h>
#include <SDL_events.h>
#include <SDL_keyboard.h>
#include <SDL_keycode.h>
#include <SDL_rect.h>
#include <SDL_render.h>
#include <SDL_stdinc.h>
#include <SDL_video.h>
#include <SDL_ttf.h>

#include <sys/types.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <string>

#include <portable-file-dialogs.h>

#include "cpu/cpu.h"
#include "emulator_session.h"
#include "ppu/ppu_context.h"

namespace gameboy::ui {
struct Emulation
{
    Emulation(app::EmulatorSession& session) : session(session)
    {
    }

    app::EmulatorSession& session;
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
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");

        window_ = SDL_CreateWindow(
            "gbemu_cpp", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, emulation_.width, emulation_.height,
            SDL_WINDOW_RESIZABLE);
        if (!window_) {
            SDL_Log("Could not create main window: %s", SDL_GetError());
            emulation_.stop = true;
            return;
        }
        SDL_SetWindowMinimumSize(window_, ppu::PPUContext::XRES, ppu::PPUContext::YRES);

        renderer_ = create_renderer(window_, "main");
        if (!renderer_) {
            emulation_.stop = true;
            return;
        }
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
        screen_ = SDL_CreateRGBSurface(
            0, ppu::PPUContext::XRES, ppu::PPUContext::YRES, 32, 0x00FF0000, 0x0000FF00, 0x000000FF,
            0xFF000000);

        debug_window_ = SDL_CreateWindow(
            "gbemu_cpp tiles", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 16 * 8 * GameboyUI::SCALE,
            32 * 8 * GameboyUI::SCALE, SDL_WINDOW_RESIZABLE);
        if (!debug_window_) {
            SDL_Log("Could not create debug window: %s", SDL_GetError());
            emulation_.stop = true;
            return;
        }
        SDL_SetWindowMinimumSize(debug_window_, DEBUG_WIDTH, DEBUG_HEIGHT);

        debug_renderer_ = create_renderer(debug_window_, "debug");
        if (!debug_renderer_) {
            emulation_.stop = true;
            return;
        }
        SDL_SetRenderDrawBlendMode(debug_renderer_, SDL_BLENDMODE_BLEND);

        texture_ = SDL_CreateTexture(
            renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, ppu::PPUContext::XRES,
            ppu::PPUContext::YRES);
        if (!texture_) {
            SDL_Log("Could not create main texture: %s", SDL_GetError());
            emulation_.stop = true;
            return;
        }

        debug_texture_ = SDL_CreateTexture(
            debug_renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING, DEBUG_WIDTH, DEBUG_HEIGHT);
        if (!debug_texture_) {
            SDL_Log("Could not create debug texture: %s", SDL_GetError());
            emulation_.stop = true;
            return;
        }

        debug_screen_ = SDL_CreateRGBSurface(
            0, DEBUG_WIDTH, DEBUG_HEIGHT, 32, 0x00FF0000, 0x0000FF00, 0x000000FF, 0xFF000000);

        int x, y;

        SDL_GetWindowPosition(window_, &x, &y);
        SDL_SetWindowPosition(debug_window_, x + emulation_.width + 10, y);
        update_window_title();
    }

    void ui_update()
    {
        auto* cpu = emulation_.session.cpu();
        if (!cpu) {
            render_menu();
            return;
        }

        const auto& ppu_context = cpu->memory().ppu().context();
        if (prev_frame_ != ppu_context.current_frame) {
            update_main(*cpu);
            update_debug_window(*cpu);
        }
        prev_frame_ = ppu_context.current_frame;
    }

    void wait_for_events()
    {
        SDL_Event e;
        while (SDL_PollEvent(&e) > 0) {
            if (e.type == SDL_KEYDOWN) {
                if (handle_command_key(e.key.keysym)) {
                    continue;
                }
                on_key(true, e.key.keysym);
            }

            if (e.type == SDL_KEYUP) {
                on_key(false, e.key.keysym);
            }

            if (e.type == SDL_WINDOWEVENT && e.window.event == SDL_WINDOWEVENT_CLOSE) {
                emulation_.stop = true;
            }

        }
    }

    ~GameboyUI()
    {
        SDL_DestroyTexture(texture_);
        SDL_DestroyTexture(debug_texture_);
        SDL_DestroyRenderer(renderer_);
        SDL_DestroyRenderer(debug_renderer_);
        SDL_DestroyWindow(window_);
        SDL_DestroyWindow(debug_window_);
        SDL_Quit();
    }

   private:
    SDL_Renderer* create_renderer(SDL_Window* window, const char* label)
    {
        constexpr std::array<uint32_t, 3> renderer_flags{
            SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC, SDL_RENDERER_ACCELERATED, SDL_RENDERER_SOFTWARE};

        for (const auto flags : renderer_flags) {
            if (auto* renderer = SDL_CreateRenderer(window, -1, flags)) {
                return renderer;
            }
        }

        SDL_Log("Could not create %s renderer: %s", label, SDL_GetError());
        return nullptr;
    }

    void delay(uint32_t ms)
    {
        SDL_Delay(ms);
    }

    bool handle_command_key(SDL_Keysym key)
    {
        const bool ctrl_pressed = (SDL_GetModState() & KMOD_CTRL) != 0;
        if ((ctrl_pressed && key.sym == SDLK_o) || key.sym == SDLK_l ||
            (!emulation_.session.running() && key.sym == SDLK_RETURN)) {
            open_rom_dialog();
            return true;
        }

        return false;
    }

    void open_rom_dialog()
    {
        const auto selection = pfd::open_file(
                                   "Open Game Boy ROM", ".",
                                   {"Game Boy ROMs", "*.gb *.gbc", "All Files", "*"},
                                   pfd::opt::none)
                                   .result();
        if (selection.empty()) {
            return;
        }

        if (emulation_.session.load_rom(selection.front())) {
            prev_frame_ = 0;
            update_window_title();
        }
    }

    void update_window_title()
    {
        std::string title = "gbemu_cpp - Press L or Ctrl+O to load ROM";
        const auto rom_path = emulation_.session.current_rom_path();
        if (!rom_path.empty()) {
            title = "gbemu_cpp - " + std::filesystem::path(rom_path).filename().string();
        }
        SDL_SetWindowTitle(window_, title.c_str());
    }

    void on_key(bool pressed, SDL_Keysym key)
    {
        auto* cpu = emulation_.session.cpu();
        if (!cpu) {
            return;
        }

        auto& gamepad = cpu->memory().gamepad();
        switch (key.sym) {
            case SDLK_z: {
                gamepad.state().b = pressed;
                break;
            }

            case SDLK_x: {
                gamepad.state().a = pressed;
                break;
            }

            case SDLK_k: {
                gamepad.state().start = pressed;
                break;
            }

            case SDLK_RETURN: {
                gamepad.state().select = pressed;
                break;
            }

            case SDLK_UP: {
                gamepad.state().up = pressed;
                break;
            }

            case SDLK_DOWN: {
                gamepad.state().down = pressed;
                break;
            }

            case SDLK_LEFT: {
                gamepad.state().left = pressed;
                break;
            }

            case SDLK_RIGHT: {
                gamepad.state().right = pressed;
                break;
            }
        }
    }

    void render_menu()
    {
        SDL_SetRenderDrawColor(renderer_, 24, 28, 35, 255);
        SDL_RenderClear(renderer_);

        int output_width = 0;
        int output_height = 0;
        SDL_GetRendererOutputSize(renderer_, &output_width, &output_height);

        const int block_width = std::max(160, output_width / 3);
        const int block_height = std::max(90, output_height / 5);
        SDL_Rect block{
            (output_width - block_width) / 2, (output_height - block_height) / 2, block_width, block_height};

        SDL_SetRenderDrawColor(renderer_, 56, 68, 88, 255);
        SDL_RenderFillRect(renderer_, &block);
        SDL_SetRenderDrawColor(renderer_, 136, 169, 112, 255);
        SDL_RenderDrawRect(renderer_, &block);

        SDL_RenderPresent(renderer_);
    }

    void update_main(cpu::CPU& cpu)
    {
        SDL_Rect rc;
        rc.x = rc.y = 0;
        rc.w = rc.h = 1;

        auto& video_buffer = cpu.memory().ppu().context().video_buffer;

        for (int line_num = 0; line_num < ppu::PPUContext::YRES; line_num++) {
            for (int x = 0; x < ppu::PPUContext::XRES; x++) {
                rc.x = x;
                rc.y = line_num;

                SDL_FillRect(screen_, &rc, video_buffer[x + (line_num * ppu::PPUContext::XRES)]);
            }
        }

        SDL_UpdateTexture(texture_, NULL, screen_->pixels, screen_->pitch);
        const auto dst = scaled_destination(renderer_, ppu::PPUContext::XRES, ppu::PPUContext::YRES);
        SDL_RenderClear(renderer_);
        SDL_RenderCopy(renderer_, texture_, NULL, &dst);
        draw_controls_overlay(cpu);
        SDL_RenderPresent(renderer_);
    }

    void draw_controls_overlay(cpu::CPU& cpu)
    {
        int output_width = 0;
        int output_height = 0;
        SDL_GetRendererOutputSize(renderer_, &output_width, &output_height);

        if (output_width < 360 || output_height < 260) {
            return;
        }

        const auto& state = cpu.memory().gamepad().state();
        const int unit = std::clamp(std::min(output_width, output_height) / 32, 12, 22);
        const int panel_width = unit * 16;
        const int panel_height = unit * 7;
        const int margin = unit;
        const int panel_x = output_width - panel_width - margin;
        const int panel_y = output_height - panel_height - margin;

        SDL_Rect panel{panel_x, panel_y, panel_width, panel_height};
        set_draw_color(18, 22, 28, 172);
        SDL_RenderFillRect(renderer_, &panel);
        set_draw_color(116, 128, 150, 140);
        SDL_RenderDrawRect(renderer_, &panel);

        const int dpad_x = panel_x + unit * 2;
        const int dpad_y = panel_y + unit * 2;
        draw_dpad_button({dpad_x + unit, dpad_y, unit, unit}, state.up, "^");
        draw_dpad_button({dpad_x + unit, dpad_y + (unit * 2), unit, unit}, state.down, "V");
        draw_dpad_button({dpad_x, dpad_y + unit, unit, unit}, state.left, "<");
        draw_dpad_button({dpad_x + (unit * 2), dpad_y + unit, unit, unit}, state.right, ">");
        draw_dpad_button({dpad_x + unit, dpad_y + unit, unit, unit}, state.up || state.down || state.left || state.right);

        const int b_x = panel_x + unit * 10;
        const int a_x = panel_x + unit * 13;
        const int ab_y = panel_y + unit * 2;
        draw_button_circle(b_x, ab_y + unit, unit, state.b);
        draw_button_circle(a_x, ab_y, unit, state.a);
        draw_text("Z", b_x - (unit / 3), ab_y + unit - (unit / 3), std::max(2, unit / 8), label_color(state.b));
        draw_text("X", a_x - (unit / 3), ab_y - (unit / 3), std::max(2, unit / 8), label_color(state.a));

        const int system_y = panel_y + unit * 5;
        draw_system_button({panel_x + unit * 6, system_y, unit * 2, unit / 2}, state.select);
        draw_system_button({panel_x + unit * 9, system_y, unit * 2, unit / 2}, state.start);
        draw_text("ENT", panel_x + unit * 6, system_y + unit, std::max(1, unit / 10), label_color(state.select));
        draw_text("K", panel_x + unit * 9 + (unit / 2), system_y + unit, std::max(1, unit / 10), label_color(state.start));
    }

    void draw_dpad_button(SDL_Rect rect, bool pressed, std::string_view label = "")
    {
        if (pressed) {
            set_draw_color(136, 169, 112, 230);
        } else {
            set_draw_color(198, 205, 215, 180);
        }
        SDL_RenderFillRect(renderer_, &rect);
        set_draw_color(20, 24, 30, 220);
        SDL_RenderDrawRect(renderer_, &rect);

        if (!label.empty()) {
            const int scale = std::max(1, rect.w / 10);
            const int label_width = static_cast<int>(label.size()) * 6 * scale;
            draw_text(
                label, rect.x + ((rect.w - label_width) / 2), rect.y + ((rect.h - (7 * scale)) / 2), scale,
                label_color(pressed));
        }
    }

    void draw_system_button(SDL_Rect rect, bool pressed)
    {
        if (pressed) {
            set_draw_color(136, 169, 112, 230);
        } else {
            set_draw_color(198, 205, 215, 150);
        }
        SDL_RenderFillRect(renderer_, &rect);
        set_draw_color(20, 24, 30, 220);
        SDL_RenderDrawRect(renderer_, &rect);
    }

    void draw_button_circle(int cx, int cy, int radius, bool pressed)
    {
        if (pressed) {
            set_draw_color(136, 169, 112, 235);
        } else {
            set_draw_color(177, 63, 85, 200);
        }

        for (int dy = -radius; dy <= radius; dy++) {
            const int dx = static_cast<int>(std::sqrt((radius * radius) - (dy * dy)));
            SDL_RenderDrawLine(renderer_, cx - dx, cy + dy, cx + dx, cy + dy);
        }

        set_draw_color(20, 24, 30, 230);
        for (int dy = -radius; dy <= radius; dy++) {
            const int dx = static_cast<int>(std::sqrt((radius * radius) - (dy * dy)));
            if (dy == -radius || dy == radius || dx <= 1) {
                SDL_RenderDrawLine(renderer_, cx - dx, cy + dy, cx + dx, cy + dy);
            }
        }
    }

    void set_draw_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a)
    {
        SDL_SetRenderDrawColor(renderer_, r, g, b, a);
    }

    SDL_Color label_color(bool pressed)
    {
        if (pressed) {
            return SDL_Color{18, 22, 28, 255};
        }
        return SDL_Color{245, 247, 250, 230};
    }

    void draw_text(std::string_view text, int x, int y, int scale, SDL_Color color)
    {
        SDL_SetRenderDrawColor(renderer_, color.r, color.g, color.b, color.a);
        int cursor_x = x;
        for (const char c : text) {
            draw_glyph(c, cursor_x, y, scale);
            cursor_x += 6 * scale;
        }
    }

    void draw_glyph(char c, int x, int y, int scale)
    {
        const auto glyph = glyph_rows(c);
        for (int row = 0; row < 7; row++) {
            for (int col = 0; col < 5; col++) {
                if ((glyph[row] & (1 << (4 - col))) == 0) {
                    continue;
                }
                SDL_Rect pixel{x + (col * scale), y + (row * scale), scale, scale};
                SDL_RenderFillRect(renderer_, &pixel);
            }
        }
    }

    std::array<uint8_t, 7> glyph_rows(char c)
    {
        switch (c) {
            case '<':
                return {0b00010, 0b00100, 0b01000, 0b10000, 0b01000, 0b00100, 0b00010};
            case '>':
                return {0b01000, 0b00100, 0b00010, 0b00001, 0b00010, 0b00100, 0b01000};
            case '^':
                return {0b00100, 0b01010, 0b10001, 0b00100, 0b00100, 0b00100, 0b00100};
            case 'A':
                return {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001};
            case 'B':
                return {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110};
            case 'D':
                return {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110};
            case 'E':
                return {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111};
            case 'K':
                return {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001};
            case 'N':
                return {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001};
            case 'T':
                return {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100};
            case 'V':
                return {0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b01010, 0b00100};
            case 'X':
                return {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001};
            case 'Z':
                return {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111};
            default:
                return {0, 0, 0, 0, 0, 0, 0};
        }
    }

    void update_debug_window(cpu::CPU& cpu)
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
                display_tile(cpu, debug_screen_, addr, tile_num, x_draw + (x * SCALE), y_draw + (y * SCALE));
                x_draw += (8 * SCALE);
                tile_num++;
            }

            y_draw += (8 * SCALE);
            x_draw = 0;
        }

        SDL_UpdateTexture(debug_texture_, NULL, debug_screen_->pixels, debug_screen_->pitch);
        const auto dst = scaled_destination(debug_renderer_, DEBUG_WIDTH, DEBUG_HEIGHT);
        SDL_RenderClear(debug_renderer_);
        SDL_RenderCopy(debug_renderer_, debug_texture_, NULL, &dst);
        SDL_RenderPresent(debug_renderer_);
    }

    SDL_Rect scaled_destination(SDL_Renderer* renderer, int source_width, int source_height)
    {
        int output_width = 0;
        int output_height = 0;
        SDL_GetRendererOutputSize(renderer, &output_width, &output_height);

        const int scale = std::max(1, std::min(output_width / source_width, output_height / source_height));
        const int scaled_width = source_width * scale;
        const int scaled_height = source_height * scale;

        return SDL_Rect{
            (output_width - scaled_width) / 2, (output_height - scaled_height) / 2, scaled_width, scaled_height};
    }

    void display_tile(cpu::CPU& cpu, SDL_Surface* surface, uint16_t start_location, uint16_t tile_num, int x, int y)
    {
        SDL_Rect rc;
        auto& memory = cpu.memory();
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

    SDL_Window* window_{nullptr};
    SDL_Window* debug_window_{nullptr};

    SDL_Surface* screen_{nullptr};
    SDL_Surface* debug_screen_{nullptr};

    SDL_Renderer* renderer_{nullptr};
    SDL_Renderer* debug_renderer_{nullptr};

    SDL_Texture* texture_{nullptr};
    SDL_Texture* debug_texture_{nullptr};

    Emulation& emulation_;

    uint32_t prev_frame_{0};

    static constexpr uint32_t SCALE{5};
    static constexpr int DEBUG_WIDTH{(16 * 8 * SCALE) + (16 * SCALE)};
    static constexpr int DEBUG_HEIGHT{(32 * 8 * SCALE) + (64 * SCALE)};
    static constexpr uint32_t GRAY_COLOR{0xFF111111};

    static constexpr uint32_t TILE_COLORS[4] = {0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000};
};
}  // namespace gameboy::ui
