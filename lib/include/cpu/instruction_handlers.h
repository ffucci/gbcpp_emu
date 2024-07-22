#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "mmu/mmu.h"
#include "cpu/cpucontext.h"
#include "cpu/instructions.h"
#include "utils/logger.h"

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

inline void nop_handler(CPUContext& ctx, memory::MMU& memory)
{
}

bool check_cond(CPUContext& context);
void ld_handler(CPUContext& ctx, memory::MMU& memory);
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

// ************************* SPECIAL INSTRUCTIONS ******************************** //
inline void rlca_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t u = ctx.registers.a;
    bool c = (u >> 7) & 1;
    u = (u << 1) | c;  // 00000111 -> (00001110) | 0;
    ctx.registers.a = u;
    cpu_set_flag(ctx.registers.f, u == 0, 0, 0, c);
}

inline void rrca_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t bit = ctx.registers.a & 1;
    ctx.registers.a >>= 1;
    ctx.registers.a |= (bit << 7);  // to wrap around

    cpu_set_flag(ctx.registers.f, 0, 0, 0, bit);
}

inline void rla_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t u = ctx.registers.a;
    uint8_t cf = ctx.registers.c_flag();
    uint8_t c = (u >> 7) & 1;
    ctx.registers.a = (u << 1) | cf;
    cpu_set_flag(ctx.registers.f, 0, 0, 0, c);
}

inline void rra_handler(CPUContext& ctx, memory::MMU& memory)
{
    uint8_t cf = ctx.registers.c_flag();
    uint8_t new_carry = ctx.registers.a & 0x1;
    ctx.registers.a >>= 1;
    ctx.registers.a |= (cf << 7);

    cpu_set_flag(ctx.registers.f, 0, 0, 0, new_carry);
}

inline void stop_handler(CPUContext& ctx, memory::MMU& memory)
{
    throw std::runtime_error("STOPPING....");
}

inline void daa_handler(CPUContext& ctx, memory::MMU& memory)
{
    int8_t u = 0;
    int fc = 0;

    auto& regs = ctx.registers;
    if (regs.h_flag() || (!regs.n_flag() && (regs.a & 0xF) > 9)) {
        u = 6;
    }

    if (regs.c_flag() || (!regs.n_flag() && regs.a > 0x99)) {
        u |= 0x60;
        fc = 1;
    }

    regs.a += regs.n_flag() ? -u : u;

    cpu_set_flag(ctx.registers.f, regs.a == 0, -1, 0, fc);
}

inline void cpl_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.registers.a = ~ctx.registers.a;
    cpu_set_flag(ctx.registers.f, -1, 1, 1, -1);
}

inline void scf_handler(CPUContext& ctx, memory::MMU& memory)
{
    cpu_set_flag(ctx.registers.f, -1, 0, 0, 1);
}

inline void ccf_handler(CPUContext& ctx, memory::MMU& memory)
{
    cpu_set_flag(ctx.registers.f, -1, 0, 0, ctx.registers.c_flag() ^ 1);
}

inline void halt_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.state = CPUState::HALT;
}

void di_handler(CPUContext& ctx, memory::MMU& memory);
inline void ei_handler(CPUContext& ctx, memory::MMU& memory)
{
}

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

    // **** SPECIAL ****
    table_[std::to_underlying(InstructionType::RLCA)] = rlca_handler;
    table_[std::to_underlying(InstructionType::RRCA)] = rrca_handler;
    table_[std::to_underlying(InstructionType::RLA)] = rla_handler;
    table_[std::to_underlying(InstructionType::RRA)] = rra_handler;

    table_[std::to_underlying(InstructionType::DAA)] = daa_handler;
    table_[std::to_underlying(InstructionType::CPL)] = cpl_handler;
    table_[std::to_underlying(InstructionType::SCF)] = scf_handler;
    table_[std::to_underlying(InstructionType::CCF)] = ccf_handler;
    table_[std::to_underlying(InstructionType::HALT)] = halt_handler;

    table_[std::to_underlying(InstructionType::DI)] = di_handler;
    table_[std::to_underlying(InstructionType::EI)] = ei_handler;

    return table_;
}

};  // namespace gameboy::cpu