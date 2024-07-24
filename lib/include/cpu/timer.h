#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include "cpu/interrupt_type.h"

namespace gameboy::cpu::timer {
struct TimerContext
{
    uint16_t div{0xABCC};
    uint8_t tima{0};
    uint8_t tma{0};
    uint8_t tac{0};
};

class Timer
{
   public:
    Timer() = default;

    void tick(uint8_t& interrupt_flags);

    void write(uint16_t address, uint8_t value);

    auto read(uint16_t address) const -> uint8_t;

   private:
    TimerContext context_{};
    static constexpr std::array<uint8_t, 4> bitmap_ = {9, 3, 5, 7};
};
}  // namespace gameboy::cpu::timer