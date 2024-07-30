#pragma once

#include <cstdint>
namespace gameboy::ppu {
struct OAMEntry
{
    int8_t y;
    int8_t x;
    int8_t tile;

    unsigned f_cgb_palette : 3;
    unsigned f_cgb_vram_bank : 1;
    unsigned f_dmg_palette : 1;
    unsigned f_x_flip : 1;
    unsigned f_y_flip : 1;
    unsigned f_priority : 1;
};
}  // namespace gameboy::ppu