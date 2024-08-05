#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "io/lcd.h"
#include "ppu/oam_data.h"
namespace gameboy::ppu {

enum class PPUFetchState : int8_t
{
    Tile,
    Data0,
    Data1,
    Idle,
    Push
};

struct Entry
{
    uint32_t color;
};

using PixelQueue = std::queue<Entry>;

struct PixelQueueContext
{
    PPUFetchState fetch_state;
    PixelQueue pixel_queue;
    uint8_t line_x{0};
    uint8_t pushed_x{0};
    uint8_t fetch_x{0};
    uint8_t bgw_fetch_data[3];

    static constexpr uint8_t OAM_DATA_SIZE{6};
    uint8_t fetch_entry_data[OAM_DATA_SIZE];

    uint8_t map_y;
    uint8_t map_x;
    uint8_t tile_y;
    uint8_t fifo_x;
};

struct PPUContext
{
    static constexpr uint8_t OAM_RAM_SIZE{40};
    static constexpr uint32_t VRAM_SIZE{0x2000};

    std::array<OAMEntry, OAM_RAM_SIZE> oam_ram{};
    std::array<std::byte, VRAM_SIZE> vram{};

    uint32_t current_frame{};
    uint32_t line_ticks{};

    std::vector<uint32_t> video_buffer{};

    PixelQueueContext pfc;
    static constexpr uint16_t YRES{144};
    static constexpr uint16_t XRES{160};

    void process(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& on_read)
    {
        const auto& context = lcd.context();
        pfc.map_x = pfc.fetch_x + context.scroll_x;
        pfc.map_y = context.ly + context.scroll_y;
        pfc.tile_y = 2 * ((context.ly + context.scroll_y) % 8);

        if (!(line_ticks & 0x1)) {
            pipeline_fetch(lcd, std::move(on_read));
        }

        push_pixel(context.scroll_x, context.ly);
    }

    void reset()
    {
        while (!pfc.pixel_queue.empty()) {
            pfc.pixel_queue.pop();
        }
    }

   private:
    void push_pixel(uint8_t scroll_x, uint8_t ly)
    {
        if (pfc.pixel_queue.size() > 8) {
            auto pixel_data = pfc.pixel_queue.front();
            pfc.pixel_queue.pop();

            if (pfc.line_x >= (scroll_x % 8)) {
                video_buffer[pfc.pushed_x + ly * XRES] = pixel_data.color;
                pfc.pushed_x++;
            }

            pfc.line_x++;
        }
    }

    void pipeline_fetch(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& read)
    {
        std::invoke(handlers_[std::to_underlying(pfc.fetch_state)], this, lcd, std::move(read));
    }

    void tile_handler(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& read)
    {
        auto& context = lcd.context();
        if (context.bgw_enable()) {
            auto delta_y = (pfc.map_y / 8) * 32;
            pfc.bgw_fetch_data[0] = read(context.bg_map_area() + (pfc.map_x / 8) + delta_y);

            if (context.bgw_data_area() == 0x8800) {
                pfc.bgw_fetch_data[0] += 128;
            }
        }

        pfc.fetch_state = PPUFetchState::Data0;
        pfc.fetch_x += 8;
    }

    void data0_handler(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& read)
    {
        auto tile_x = pfc.bgw_fetch_data[0] * 16;
        pfc.bgw_fetch_data[1] = read(lcd.context().bgw_data_area() + tile_x + pfc.tile_y);

        pfc.fetch_state = PPUFetchState::Data1;
    }

    void data1_handler(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& read)
    {
        auto tile_x = pfc.bgw_fetch_data[0] * 16;
        pfc.bgw_fetch_data[2] = read(lcd.context().bgw_data_area() + tile_x + pfc.tile_y + 1);

        pfc.fetch_state = PPUFetchState::Idle;
    }

    void idle_handler(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& read)
    {
        pfc.fetch_state = PPUFetchState::Push;
    }

    bool add_to_queue(const lcd::LCDContext& lcd_context)
    {
        if (pfc.pixel_queue.size() > 8) {
            return false;
        }

        int x = pfc.fetch_x - (8 - (lcd_context.scroll_x % 8));

        for (int i = 0; i < 8; ++i) {
            auto bit = 7 - i;
            uint8_t hi = !!(pfc.bgw_fetch_data[2] & (1 << bit)) << 1;
            uint8_t lo = !!(pfc.bgw_fetch_data[1] & (1 << bit));
            uint32_t color = lcd_context.bg_colors[hi | lo];

            if (x >= 0) {
                pfc.pixel_queue.emplace(color);
                pfc.fifo_x++;
            }
        }

        return true;
    }

    void push_handler(lcd::LCD& lcd, std::function<uint8_t(uint16_t)>&& read)
    {
        if (add_to_queue(lcd.context())) {
            pfc.fetch_state = PPUFetchState::Tile;
        }
    }

    using PipelineHandler = void (PPUContext::*)(lcd::LCD&, std::function<uint8_t(uint16_t)>&&);
    std::array<PipelineHandler, std::to_underlying(PPUFetchState::Push) + 1> handlers_{
        &PPUContext::tile_handler, &PPUContext::data0_handler, &PPUContext::data1_handler, &PPUContext::idle_handler,
        &PPUContext::push_handler};
};
}  // namespace gameboy::ppu