#pragma once

#include <sys/types.h>
#include <array>
#include <cstdint>
#include <functional>
#include <queue>
#include <utility>
#include <vector>

#include "io/lcd.h"
#include "ppu/oam_data.h"
#include "ppu/oam_freelist.h"
#include "utils/logger.h"
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

    uint8_t map_y;
    uint8_t map_x;
    uint8_t tile_y;
    uint8_t fifo_x;

    static constexpr uint8_t OAM_DATA_SIZE{6};
    uint8_t fetch_entry_data[OAM_DATA_SIZE];
};

struct PPUContext
{
    static constexpr uint8_t OAM_RAM_SIZE{40};
    static constexpr uint32_t VRAM_SIZE{0x2000};
    static constexpr uint16_t YRES{144};
    static constexpr uint16_t XRES{160};

    std::array<OAMEntry, OAM_RAM_SIZE> oam_ram = {};
    std::array<std::byte, VRAM_SIZE> vram = {};
    OAMFreeList<10> oam_freelist{};

    uint32_t current_frame{};
    uint32_t line_ticks{};

    std::vector<uint32_t> video_buffer{};
    PixelQueueContext pfc;

    void process(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& on_read)
    {
        const auto& context = lcd.context();
        pfc.map_x = pfc.fetch_x + context.scroll_x;
        pfc.map_y = context.ly + context.scroll_y;
        pfc.tile_y = 2 * ((context.ly + context.scroll_y) % 8);

        if (!(line_ticks & 0x1)) {
            pipeline_fetch(lcd, on_read);
        }

        push_pixel(context.scroll_x, context.ly);
    }

    void fifo_reset()
    {
        while (!pfc.pixel_queue.empty()) {
            pfc.pixel_queue.pop();
        }
    }

    void load_line_sprites(lcd::LCD& lcd)
    {
        const auto& context = lcd.context();
        uint8_t sprite_height = context.obj_height();

        uint8_t curr_y = context.ly;
        oam_freelist.clear();
        for (int i = 0; i < OAM_RAM_SIZE; ++i) {
            auto entry = oam_ram[i];

            if (!entry.x) {
                continue;
            }

            if (oam_freelist.sprite_lines() >= 10) {
                break;
            }

            if (entry.y <= curr_y + 16 && entry.y + sprite_height > curr_y + 16) {
                // insert into the freelist
                auto stop = oam_freelist.insert(entry);
            }
        }
    }

   private:
    void push_pixel(uint8_t scroll_x, uint8_t ly)
    {
        if (pfc.pixel_queue.size() > 8) {
            auto pixel_data = pfc.pixel_queue.front();
            pfc.pixel_queue.pop();

            if (pfc.line_x >= (scroll_x % 8)) {
                video_buffer[pfc.pushed_x + (ly * XRES)] = pixel_data.color;
                pfc.pushed_x++;
            }

            pfc.line_x++;
        }
    }

    void load_sprite_data(
        uint8_t offset, const lcd::LCDContext& lcd_context, const std::function<uint8_t(uint16_t)>& read)
    {
        int ly = lcd_context.ly;
        uint8_t sprite_height = lcd_context.obj_height();
        auto& fetched_entries = oam_freelist.fetched_entries_ref();
        for (int i = 0; i < oam_freelist.fetched_entry_count(); ++i) {
            // 16 is the y offset of 16 bytes
            uint8_t ty = 2 * ((ly + 16) - fetched_entries[i].y);
            if (fetched_entries[i].f_y_flip) {
                ty = ((sprite_height * 2) - 2) - ty;  // flip on the y-axis
            }

            uint8_t tile_idx = fetched_entries[i].tile;
            constexpr uint16_t BASE_OFFSET = 0x8000;

            if (sprite_height == 16) {
                tile_idx &= ~(1);
            }
            pfc.fetch_entry_data[(2 * i) + offset] = read(BASE_OFFSET + (tile_idx * 16) + ty + offset);
        }
    }

    void pipeline_fetch(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& read)
    {
        std::invoke(handlers_[std::to_underlying(pfc.fetch_state)], this, lcd, read);
    }

    void tile_handler(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& read)
    {
        auto& context = lcd.context();
        oam_freelist.fetched_entry_count() = 0;
        if (context.bgw_enable()) {
            uint16_t delta_y = (pfc.map_y / 8) * 32;
            pfc.bgw_fetch_data[0] = read(context.bg_map_area() + (pfc.map_x / 8) + delta_y);

            if (context.bgw_data_area() == 0x8800) {
                pfc.bgw_fetch_data[0] += 128;
            }
        }

        if (context.obj_enable() && oam_freelist.has_freelist_head()) {
            oam_freelist.load_sprite_tile(pfc.fetch_x, context.scroll_x);
        }

        pfc.fetch_state = PPUFetchState::Data0;
        pfc.fetch_x += 8;
    }

    void data0_handler(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& read)
    {
        uint16_t tile_x = pfc.bgw_fetch_data[0] * 16;
        pfc.bgw_fetch_data[1] = read(lcd.context().bgw_data_area() + tile_x + pfc.tile_y);
        load_sprite_data(0, lcd.context(), read);
        pfc.fetch_state = PPUFetchState::Data1;
    }

    void data1_handler(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& read)
    {
        uint16_t tile_x = pfc.bgw_fetch_data[0] * 16;
        pfc.bgw_fetch_data[2] = read(lcd.context().bgw_data_area() + tile_x + pfc.tile_y + 1);
        load_sprite_data(1, lcd.context(), read);
        pfc.fetch_state = PPUFetchState::Idle;
    }

    void idle_handler(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& read)
    {
        pfc.fetch_state = PPUFetchState::Push;
    }

    uint32_t fetch_sprite_pixels(int bit, uint32_t color, uint8_t bg_color, const lcd::LCDContext& lcd_context)
    {
        auto& fetched_entries = oam_freelist.fetched_entries_ref();
        auto& logger = logger::Logger::instance();

        for (int i = 0; i < oam_freelist.fetched_entry_count(); ++i) {
            int sprite_x = (fetched_entries[i].x - 8) + (lcd_context.scroll_x % 8);

            if (sprite_x + 8 < pfc.fifo_x) {
                // passed the pixel point
                continue;
            }

            int offset = pfc.fifo_x - sprite_x;
            if (offset < 0 || offset > 7) {
                // out of bounds
                continue;
            }

            bit = (7 - offset);
            if (fetched_entries[i].f_x_flip) {
                bit = offset;
            }

            uint8_t lo = !!(pfc.fetch_entry_data[2 * i] & (1 << bit));
            uint8_t hi = !!(pfc.fetch_entry_data[(2 * i) + 1] & (1 << bit)) << 1;
            auto res = hi | lo;
            bool background_prio = fetched_entries[i].f_background_priority;
            if (!res) {
                // transparent
                continue;
            }

            if (!(background_prio) || (bg_color == 0)) {
                color = fetched_entries[i].f_dmg_palette ? lcd_context.sprite2_colors[res]
                                                         : lcd_context.sprite1_colors[res];
                if (hi | lo) {
                    break;
                }
            }
        }

        return color;
    }

    bool add_to_queue(const lcd::LCDContext& lcd_context)
    {
        if (pfc.pixel_queue.size() > 8) {
            return false;
        }

        int x = pfc.fetch_x - (8 - (lcd_context.scroll_x % 8));

        for (int i = 0; i < 8; ++i) {
            int bit = 7 - i;
            uint8_t hi = !!(pfc.bgw_fetch_data[2] & (1 << bit)) << 1;
            uint8_t lo = !!(pfc.bgw_fetch_data[1] & (1 << bit));
            uint32_t color = lcd_context.bg_colors[hi | lo];

            if (!lcd_context.bgw_enable()) {
                color = lcd_context.bg_colors[0];
            }

            // Sprites are enabled
            if (lcd_context.obj_enable()) {
                color = fetch_sprite_pixels(bit, color, hi | lo, lcd_context);
            }

            if (x >= 0) {
                pfc.pixel_queue.emplace(color);
                pfc.fifo_x++;
            }
        }

        return true;
    }

    void push_handler(lcd::LCD& lcd, const std::function<uint8_t(uint16_t)>& read)
    {
        if (add_to_queue(lcd.context())) {
            pfc.fetch_state = PPUFetchState::Tile;
        }
    }

    using PipelineHandler = void (PPUContext::*)(lcd::LCD&, const std::function<uint8_t(uint16_t)>&);
    std::array<PipelineHandler, std::to_underlying(PPUFetchState::Push) + 1> handlers_{
        &PPUContext::tile_handler, &PPUContext::data0_handler, &PPUContext::data1_handler, &PPUContext::idle_handler,
        &PPUContext::push_handler};
};
}  // namespace gameboy::ppu