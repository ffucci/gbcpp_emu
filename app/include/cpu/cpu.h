#pragma once

#include <cstdint>

#include "instructions.h"

namespace gameboy::cpu {

struct CPUState
{
    uint8_t a;

    // Highest 4 bytes are the Z, N, H, C flags
    uint8_t f;
    uint8_t b;
    uint8_t c;
    uint8_t d;
    uint8_t e;
    uint8_t h;
    uint8_t l;
    uint16_t pc;  // Program Counter
    uint16_t sp;  // Stack pointer
};

inline auto get_initial_state() -> CPUState
{
    // DMG
    return {0x1, 0, 0x0, 0x13, 0x00, 0xD8, 0x01, 0x4d, 0x100, 0xFFFE};
}

class CPU
{
   public:
    CPU(CPUState initial_state) : state_(std::move(initial_state))
    {
    }

    void run()
    {
    }

   private:
    void next_step() noexcept
    {
    }

    CPUState state_;
    static constexpr auto instruction_set_ = initialize_instruction_set();
};
}  // namespace gameboy::cpu
