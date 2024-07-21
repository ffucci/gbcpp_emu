#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "mmu/mmu.h"
#include "cpu/cpucontext.h"
#include "cpu/instructions.h"
#include "utils/logger.h"

#define BIT_SET(a, n, on)   \
    {                       \
        if (on)             \
            a |= (1 << n);  \
        else                \
            a &= ~(1 << n); \
    }

namespace gameboy::cpu {

using Handler = void (*)(CPUContext& context, memory::MMU&);
using ExecutorsTable = std::array<Handler, NUM_INSTRUCTION_TYPES>;

inline void stack_push(CPUContext& context, memory::MMU& memory, uint8_t value)
{
    context.registers.sp--;
    memory.write(context.registers.sp, value);
}

inline void stack_push16(CPUContext& context, memory::MMU& memory, uint16_t value)
{
    stack_push(context, memory, (value >> 8) & 0xFF);
    stack_push(context, memory, value & 0xFF);
}

inline auto stack_pop(CPUContext& context, memory::MMU& memory) -> uint8_t
{
    return memory.read(context.registers.sp++);
}

inline auto stack_pop16(CPUContext& context, memory::MMU& memory) -> uint8_t
{
    const auto lo = stack_pop(context, memory);
    const auto hi = stack_pop(context, memory);
    return lo | (hi << 8);
}

inline void none_handler(CPUContext& ctx, memory::MMU& memory)
{
    throw std::runtime_error("CANNOT EXECUTE");
}

inline void cpu_set_flag_late(uint8_t& flags, uint8_t zero, uint8_t n, uint8_t half, uint8_t carry)
{
    const auto bz = (zero << 7);
    const auto bn = (n << 6);
    const auto bh = (half << 5);
    const auto bc = (carry << 4);
    auto& logger = logger::Logger::instance();
    logger.log("z: {:08b}, n: {:08b}, h: {:08b}, c: {:08b}", zero, n, half, carry);
    logger.log("bz: {:08b}, bn: {:08b}, bh: {:08b}, bc: {:08b}", bz, bn, bh, bc);
    flags |= (flags & ~bz) | (-(zero != 0xFF) & bz);
    logger.log("flags {:08b}", flags);

    flags |= (flags & ~bn) | (-(n != 0xFF) & bn);
    logger.log("flags {:08b}", flags);

    flags |= (flags & ~bh) | (-(half != 0xFF) & bh);
    logger.log("flags {:08b}", flags);

    flags |= (flags & ~bc) | (-(carry != 0xFF) & bc);
    logger.log("flags {:08b}", flags);
}

inline void cpu_set_flag(uint8_t& flags, char zero, char n, char half, char carry)
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

inline void cpu_set_flag_macro(uint8_t& flags, char z, char n, char h, char c)
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

inline void nop_handler(CPUContext& ctx, memory::MMU& memory)
{
}

bool check_cond(CPUContext& context);
void ld_handler(CPUContext& ctx, memory::MMU& memory);

void di_handler(CPUContext& ctx, memory::MMU& memory);
void ldh_handler(CPUContext& ctx, memory::MMU& memory);

// ************************* STACK INSTRUCTIONS *************************** //

// Stack functions
void push_handler(CPUContext& ctx, memory::MMU& memory);
void pop_handler(CPUContext& ctx, memory::MMU& memory);

// ************************* JUMP/CALL/RET INSTRUCTIONS *************************** //
void jump_to_addr(CPUContext& ctx, memory::MMU& memory, uint16_t addr, bool push_pc);
void jp_handler(CPUContext& ctx, memory::MMU& memory);
void jr_handler(CPUContext& ctx, memory::MMU& memory);
void call_handler(CPUContext& ctx, memory::MMU& memory);
void ret_handler(CPUContext& ctx, memory::MMU& memory);
// returning from interrupt
void reti_handler(CPUContext& ctx, memory::MMU& memory);
void rst_handler(CPUContext& ctx, memory::MMU& memory);

// ************************* INC/DEC INSTRUCTIONS ******************************** //
void inc_handler(CPUContext& ctx, memory::MMU& memory);
void dec_handler(CPUContext& ctx, memory::MMU& memory);

// ************************* ADD/SUB INSTRUCTIONS ******************************** //
void add_handler(CPUContext& ctx, memory::MMU& mmu);
void adc_handler(CPUContext& ctx, memory::MMU& mmu);
void sub_handler(CPUContext& ctx, memory::MMU& memory);
void sbc_handler(CPUContext& ctx, memory::MMU& memory);

// ************************* LOGICAL INSTRUCTIONS ******************************** //
void and_handler(CPUContext& ctx, memory::MMU& memory);
void xor_handler(CPUContext& ctx, memory::MMU& memory);
void or_handler(CPUContext& ctx, memory::MMU& memory);
void cp_handler(CPUContext& ctx, memory::MMU& memory);

// ************************* PREFIX CB ******************************** //
void cb_handler(CPUContext& ctx, memory::MMU& memory);

constexpr auto make_executors_table() -> const ExecutorsTable
{
    ExecutorsTable table_{};
    std::fill(begin(table_), end(table_), none_handler);
    table_[std::to_underlying(InstructionType::NOP)] = nop_handler;
    table_[std::to_underlying(InstructionType::JP)] = jp_handler;
    table_[std::to_underlying(InstructionType::JR)] = jr_handler;
    table_[std::to_underlying(InstructionType::INC)] = inc_handler;
    table_[std::to_underlying(InstructionType::DEC)] = dec_handler;

    // The details of these instructions need to be understood better
    table_[std::to_underlying(InstructionType::ADD)] = add_handler;
    table_[std::to_underlying(InstructionType::ADC)] = adc_handler;
    table_[std::to_underlying(InstructionType::SUB)] = sub_handler;
    table_[std::to_underlying(InstructionType::SBC)] = sbc_handler;

    table_[std::to_underlying(InstructionType::XOR)] = xor_handler;
    table_[std::to_underlying(InstructionType::OR)] = or_handler;
    table_[std::to_underlying(InstructionType::AND)] = and_handler;
    table_[std::to_underlying(InstructionType::CP)] = cp_handler;
    // CB prefix
    table_[std::to_underlying(InstructionType::CB)] = cb_handler;

    // Going to subroutines opcodes (CALL, RET, ...)
    table_[std::to_underlying(InstructionType::CALL)] = call_handler;
    table_[std::to_underlying(InstructionType::RET)] = ret_handler;
    table_[std::to_underlying(InstructionType::RETI)] = reti_handler;
    table_[std::to_underlying(InstructionType::RST)] = rst_handler;
    table_[std::to_underlying(InstructionType::LD)] = ld_handler;
    table_[std::to_underlying(InstructionType::LDH)] = ldh_handler;
    table_[std::to_underlying(InstructionType::PUSH)] = push_handler;
    table_[std::to_underlying(InstructionType::POP)] = pop_handler;
    table_[std::to_underlying(InstructionType::DI)] = di_handler;
    return table_;
}

};  // namespace gameboy::cpu