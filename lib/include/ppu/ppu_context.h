#pragma once

#include <array>
#include <cstdint>
#include "ppu/oam_data.h"

namespace gameboy::ppu {
struct PPUContext
{
    static constexpr uint8_t OAM_RAM_SIZE{40};
    static constexpr uint32_t VRAM_SIZE{0x2000};

    std::array<OAMEntry, OAM_RAM_SIZE> oam_ram{};
    std::array<std::byte, VRAM_SIZE> vram{};
};
}  // namespace gameboy::ppu