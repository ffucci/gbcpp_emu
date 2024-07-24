#pragma once

#include <cstdint>

namespace gameboy::cpu {
enum class InterruptType : uint8_t
{
    VBLANK = 1,
    LCD_STAT = 2,
    TIMER = 4,
    SERIAL = 8,
    JOYPAD = 16
};
}