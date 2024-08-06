#pragma once

#include <cstdint>
namespace gameboy::ppu {
struct OAMEntry
{
    uint8_t y;
    uint8_t x;
    uint8_t tile;

    uint8_t f_cgb_palette : 3;
    uint8_t f_cgb_vram_bank : 1;
    uint8_t f_dmg_palette : 1;
    uint8_t f_x_flip : 1;
    uint8_t f_y_flip : 1;
    uint8_t f_background_priority : 1;
};
}  // namespace gameboy::ppu