#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "mmu/mmu.h"
#include "cpu/cpucontext.h"
#include "cpu/instructions.h"

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

inline void cpu_set_flag(uint8_t& flags, uint8_t zero, uint8_t n, uint8_t half, uint8_t carry)
{
    const auto bz = (zero << 7);
    const auto bn = (n << 6);
    const auto bh = (half << 5);
    const auto bc = (carry << 4);

    flags = (flags & ~bz) | (-(zero != 0xFF) & bz);
    flags = (flags & ~bn) | (-(n != 0xFF) & bn);
    flags = (flags & ~bh) | (-(half != 0xFF) & bh);
    flags = (flags & ~bc) | (-(carry != 0xFF) & bc);
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
    table_[std::to_underlying(InstructionType::OR)] = add_handler;
    table_[std::to_underlying(InstructionType::AND)] = adc_handler;
    table_[std::to_underlying(InstructionType::CP)] = sub_handler;
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