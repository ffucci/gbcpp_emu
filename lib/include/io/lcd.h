#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <iostream>
#include "cpu/cpucontext.h"
#include "cpu/interrupt_type.h"
#include "utils/logger.h"

namespace gameboy::lcd {

enum class LCDMode : uint8_t
{
    HBlank = 0,
    VBlank,
    Oam,
    Transfer
};

enum class LCDStat
{
    HBlank = (1 << 3),
    VBlank = (1 << 4),
    Oam = (1 << 5),
    Lyc = (1 << 6)
};

struct LCDContext
{
    uint8_t lcdc{};  // LCD control register
    uint8_t lcds{};  // LCD status register
    uint8_t scroll_y{};
    uint8_t scroll_x{};
    uint8_t ly{0};
    uint8_t ly_compare{0};
    uint8_t dma;
    uint8_t bg_palette{};
    uint8_t obj_palette[2];
    uint8_t win_y{0x0};
    uint8_t win_x{0x0};

    // Other data
    uint32_t bg_colors[4];
    uint32_t sprite1_colors[4];
    uint32_t sprite2_colors[4];

    // LCD & PPU enable: 0 = Off; 1 = On
    // Window tile map area: 0 = 9800–9BFF; 1 = 9C00–9FFF
    // Window enable: 0 = Off; 1 = On
    // BG & Window tile data area: 0 = 8800–97FF; 1 = 8000–8FFF
    // BG tile map area: 0 = 9800–9BFF; 1 = 9C00–9FFF
    // OBJ size: 0 = 8×8; 1 = 8×16
    // OBJ enable: 0 = Off; 1 = On
    // BG & Window enable / priority [Different meaning in CGB Mode]: 0 = Off; 1 = On

    inline bool bgw_enable() const noexcept
    {
        return lcdc & (1 << 0);
    }
    inline bool obj_enable() const noexcept
    {
        return (lcdc & (1 << 1));
    }

    inline uint8_t obj_height() const noexcept
    {
        return lcdc & (1 << 2) ? 16 : 8;
    }

    inline uint16_t bg_map_area() const noexcept
    {
        return (lcdc & (1 << 3)) ? 0x9C00 : 0x9800;
    }

    inline uint16_t bgw_data_area() const noexcept
    {
        return (lcdc & (1 << 4)) ? 0x8000 : 0x8800;
    }

    inline bool win_enable() const noexcept
    {
        return lcdc & (1 << 5);
    }

    inline uint16_t win_map_area() const noexcept
    {
        return lcdc & (1 << 6) ? 0x9C00 : 0x9800;
    }

    inline bool lcd_enable() const noexcept
    {
        return lcdc & (1 << 7);
    }

    inline LCDMode lcds_mode() const noexcept
    {
        return static_cast<LCDMode>(lcds & 0b11);
    }

    inline void set_lcds_mode(LCDMode mode) noexcept
    {
        lcds &= ~0b11;
        lcds |= std::to_underlying(mode);
    }

    inline bool status_lyc_get() noexcept
    {
        return lcds & (1 << 2);
    }

    inline void status_lyc_set(bool on) noexcept
    {
        lcds ^= (-on ^ lcds) & (1 << 2);
    }

    inline bool status_interrupt_mode(LCDStat src) const noexcept
    {
        return (lcds & std::to_underlying(src)) != 0;
    }

    inline void inc_ly(cpu::CPUContext& context)
    {
        ly++;
        if (ly == ly_compare) {
            status_lyc_set(1);

            if (status_interrupt_mode(LCDStat::Lyc)) {
                context.request_interrupt(cpu::InterruptType::LCD_STAT);
            }
        } else {
            status_lyc_set(0);
        }
    }
};

class LCD
{
   public:
    LCD()
    {
        lcd_context_.lcdc = 0x91;
        lcd_context_.scroll_x = 0;
        lcd_context_.scroll_y = 0;
        lcd_context_.ly = 0;
        lcd_context_.ly_compare = 0;
        lcd_context_.bg_palette = 0xFC;
        lcd_context_.obj_palette[0] = 0xFF;
        lcd_context_.obj_palette[1] = 0xFF;
        lcd_context_.win_y = 0;
        lcd_context_.win_x = 0;
        for (size_t i = 0; i < 4; ++i) {
            lcd_context_.bg_colors[i] = colors_default[i];
            lcd_context_.sprite1_colors[i] = colors_default[i];
            lcd_context_.sprite2_colors[i] = colors_default[i];
        }
        // Sets the LCD in mode OAM
        lcd_context_.set_lcds_mode(LCDMode::Oam);
    }

    template <std::regular_invocable<uint8_t> DMACallback>
    inline void write(uint16_t address, uint8_t value, DMACallback&& callback)
    {
        uint8_t offset = (address - LCD_BASE_ADDRESS) & 0xFF;
        uint8_t* ctx_ptr = std::bit_cast<uint8_t*>(&lcd_context_);
        ctx_ptr[offset] = value;

        if (offset == 0x6) {
            std::forward<DMACallback>(callback)(value);
        }

        if (address == BGP_PALETTE_ADDR) {
            update_palette(value, 0);
            return;
        }

        if (address == OBJ_PALETTE0_ADDR) {
            update_palette(value & 0b11111100, 1);
            return;
        }

        if (address == OBJ_PALETTE1_ADDR) {
            update_palette(value & 0b11111100, 2);
            return;
        }
    }

    auto read(uint16_t address) const noexcept -> uint8_t
    {
        uint8_t offset = (address - LCD_BASE_ADDRESS) & 0xFF;
        return std::bit_cast<uint8_t*>(&lcd_context_)[offset];
    }

    auto context() -> LCDContext&
    {
        return lcd_context_;
    }

   private:
    void update_palette(uint8_t palette_data, uint8_t pal)
    {
        uint32_t* p_colors = lcd_context_.bg_colors;

        switch (pal) {
            case 1: {
                p_colors = lcd_context_.sprite1_colors;
                break;
            }

            case 2: {
                p_colors = lcd_context_.sprite2_colors;
                break;
            }
        }

        for (int i = 0; i < 4; ++i) {
            // take the palette data the cut the 2 bits
            p_colors[i] = colors_default[(palette_data >> (2 * i)) & 0b11];
        }
    }

    LCDContext lcd_context_{};
    static constexpr uint32_t colors_default[] = {0xFFFFFFFF, 0xFFAAAAAA, 0xFF555555, 0xFF000000};

    static constexpr uint16_t LCD_BASE_ADDRESS{0xFF40};
    static constexpr uint16_t BGP_PALETTE_ADDR{0xFF47};
    static constexpr uint16_t OBJ_PALETTE0_ADDR{0xFF48};
    static constexpr uint16_t OBJ_PALETTE1_ADDR{0xFF49};
};

}  // namespace gameboy::lcd