#pragma once

#include <cstdint>
#include "cpu/instructions.h"

namespace gameboy::cpu {

static constexpr uint8_t ZERO = {1 << 7};
static constexpr uint8_t N_SUB = {1 << 6};
static constexpr uint8_t HALF = {1 << 5};
static constexpr uint8_t CARRY = {1 << 4};

static constexpr bool DEBUG{false};

struct CPURegisters
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

    [[nodiscard]] std::string get_flags_string() const noexcept
    {
        std::string ret(4, '-');

        if (f & CARRY) {
            ret[3] = 'C';
        }

        if (f & HALF) {
            ret[2] = 'H';
        }

        if (f & N_SUB) {
            ret[1] = 'N';
        }

        if (f & ZERO) {
            ret[0] = 'S';
        }

        return ret;
    }
};

inline auto get_initial_state() -> CPURegisters
{
    // DMG
    // return {0x1, 0, 0x0, 0x13, 0x00, 0xD8, 0x01, 0x4d, 0x100, 0xFFFE};
    return {0x1, 0, 0x0, 0x0, 0x00, 0x0, 0x0, 0x0, 0x100, 0x0};
}

enum class CPUState
{
    HALT,
    RUNNING
};

constexpr uint16_t reverse(uint16_t n)
{
    return ((n & 0xFF00) >> 8) | ((n & 0x00FF) << 8);
}

struct CPUContext
{
    CPURegisters registers;
    uint16_t fetched_data;
    uint16_t memory_destination;
    uint8_t current_opcode;

    Instruction instruction;
    CPUState state;

    bool interrupt_masked{false};
    bool destination_is_mem{false};

    // Read registry
    auto cpu_read_reg(RegisterType rt) -> uint16_t
    {
        using gameboy::cpu::RegisterType;
        switch (rt) {
            case RegisterType::A:
                return registers.a;
            case RegisterType::F:
                return registers.f;
            case RegisterType::B:
                return registers.b;
            case RegisterType::C:
                return registers.c;
            case RegisterType::D:
                return registers.d;
            case RegisterType::E:
                return registers.e;
            case RegisterType::H:
                return registers.h;
            case RegisterType::L:
                return registers.l;

            case RegisterType::AF:
                return reverse(*((uint16_t*)&registers.a));
            case RegisterType::BC:
                return reverse(*((uint16_t*)&registers.b));
            case RegisterType::DE:
                return reverse(*((uint16_t*)&registers.d));
            case RegisterType::HL:
                return reverse(*((uint16_t*)&registers.h));
            case RegisterType::PC:
                return registers.pc;
            case RegisterType::SP:
                return registers.sp;
            default:
                return 0;
        }
    }

    void cpu_set_reg(RegisterType rt, uint16_t val)
    {
        switch (rt) {
            case RegisterType::A:
                registers.a = val & 0xFF;
                break;
            case RegisterType::F:
                registers.f = val & 0xFF;
                break;
            case RegisterType::B:
                registers.b = val & 0xFF;
                break;
            case RegisterType::C: {
                registers.c = val & 0xFF;
            } break;
            case RegisterType::D:
                registers.d = val & 0xFF;
                break;
            case RegisterType::E:
                registers.e = val & 0xFF;
                break;
            case RegisterType::H:
                registers.h = val & 0xFF;
                break;
            case RegisterType::L:
                registers.l = val & 0xFF;
                break;

            case RegisterType::AF:
                *((uint16_t*)&registers.a) = reverse(val);
                break;
            case RegisterType::BC:
                *((uint16_t*)&registers.b) = reverse(val);
                break;
            case RegisterType::DE:
                *((uint16_t*)&registers.d) = reverse(val);
                break;
            case RegisterType::HL: {
                *((uint16_t*)&registers.h) = reverse(val);
                break;
            }

            case RegisterType::PC:
                registers.pc = val;
                break;
            case RegisterType::SP:
                registers.sp = val;
                break;
            case RegisterType::None:
                break;

            default:
                throw std::runtime_error("Invalid register type...");
        }
    }

    static auto is_16_bit(RegisterType rt)
    {
        return rt >= RegisterType::AF;
    }
};

}  // namespace gameboy::cpu