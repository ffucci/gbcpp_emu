#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <iostream>

#include "cpu/instructions.h"
#include "cpu/interrupt_type.h"
#include "cpu/timer.h"

namespace gameboy::cpu {

static constexpr uint8_t ZERO = {1 << 7};
static constexpr uint8_t N_SUB = {1 << 6};
static constexpr uint8_t HALF = {1 << 5};
static constexpr uint8_t CARRY = {1 << 4};

static constexpr bool DEBUG{false};

#define BIT_SET(a, n, on)   \
    {                       \
        if (on)             \
            a |= (1 << n);  \
        else                \
            a &= ~(1 << n); \
    }

// ************************* CPU UTILITY *************************** //

// TODO: Fix since it is wrong...
inline void cpu_set_flag_late(uint8_t& flags, uint8_t zero, uint8_t n, uint8_t half, uint8_t carry)
{
    const auto bz = (zero << 7);
    const auto bn = (n << 6);
    const auto bh = (half << 5);
    const auto bc = (carry << 4);
    flags |= (flags & ~bz) | (-(zero != 0xFF) & bz);
    flags |= (flags & ~bn) | (-(n != 0xFF) & bn);
    flags |= (flags & ~bh) | (-(half != 0xFF) & bh);
    flags |= (flags & ~bc) | (-(carry != 0xFF) & bc);
}

inline void cpu_set_flag(uint8_t& flags, int8_t zero, int8_t n, int8_t half, int8_t carry)
{
    const bool is_zero = (zero != 0);
    const bool is_n = (n != 0);
    const bool is_half = (half != 0);
    const bool is_carry = (carry != 0);

    const auto mbz = (1 << 7);
    const auto mbn = (1 << 6);
    const auto mbh = (1 << 5);
    const auto mbc = (1 << 4);

    flags ^= (zero != -1 ? -1 : 0) & ((-is_zero ^ flags) & mbz);
    flags ^= (n != -1 ? -1 : 0) & ((-is_n ^ flags) & mbn);
    flags ^= (half != -1 ? -1 : 0) & ((-is_half ^ flags) & mbh);
    flags ^= (carry != -1 ? -1 : 0) & ((-is_carry ^ flags) & mbc);
}

inline void cpu_set_flag_old(uint8_t& flags, int8_t z, int8_t n, int8_t h, int8_t c)
{
    if (z != -1) {
        BIT_SET(flags, 7, z);
    }

    if (n != -1) {
        BIT_SET(flags, 6, n);
    }

    if (h != -1) {
        BIT_SET(flags, 5, h);
    }

    if (c != -1) {
        BIT_SET(flags, 4, c);
    }
}

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
            ret[0] = 'Z';
        }

        return ret;
    }

    [[nodiscard]] auto z_flag() const noexcept -> uint8_t
    {
        return (f & ZERO) ? 1 : 0;
    }

    [[nodiscard]] auto n_flag() const noexcept -> uint8_t
    {
        return (f & N_SUB) ? 1 : 0;
    }

    [[nodiscard]] auto h_flag() const noexcept -> uint8_t
    {
        return (f & HALF) ? 1 : 0;
    }

    [[nodiscard]] auto c_flag() const noexcept -> uint8_t
    {
        return (f & CARRY) ? 1 : 0;
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
    CPURegisters registers{};
    uint16_t fetched_data{};
    uint16_t memory_destination{};
    uint8_t current_opcode{};
    uint8_t interrupt_flags{0};

    Instruction instruction;
    CPUState state;
    bool master_interrupt_enabled{false};
    bool enabling_ime{false};
    bool destination_is_mem{false};
    uint64_t ticks{0};
    timer::Timer timer;

    // Read registry
    auto read_reg(RegisterType rt) -> uint16_t
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

    void set_reg(RegisterType rt, uint16_t val)
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

    auto read_reg8(RegisterType rt) -> uint8_t
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
            default:
                return 0;
        }
    }

    void set_reg8(RegisterType rt, uint8_t val)
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

            default:
                throw std::runtime_error("Invalid register type...");
        }
    }

    static auto is_16_bit(RegisterType rt)
    {
        return rt >= RegisterType::AF;
    }

    inline void request_interrupt(InterruptType interrupt)
    {
        interrupt_flags |= std::to_underlying(interrupt);
    }
};

}  // namespace gameboy::cpu