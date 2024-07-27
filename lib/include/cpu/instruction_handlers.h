#pragma once

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "cpu/instructions.h"
#include "cpu/cpucontext.h"
#include "mmu/mmu.h"

#include "utils/logger.h"

namespace gameboy::cpu {

using Handler = void (*)(CPUContext& context, memory::MMU&);
using ExecutorsTable = std::array<Handler, NUM_INSTRUCTION_TYPES>;

void stack_push(CPUContext& context, memory::MMU& memory, uint8_t value);

void stack_push16(CPUContext& context, memory::MMU& memory, uint16_t value);

auto stack_pop(CPUContext& context, memory::MMU& memory) -> uint8_t;

auto stack_pop16(CPUContext& context, memory::MMU& memory) -> uint8_t;

void none_handler(CPUContext& ctx, memory::MMU& memory);

void nop_handler(CPUContext& ctx, memory::MMU& memory);

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
void rlca_handler(CPUContext& ctx, memory::MMU& memory);

void rrca_handler(CPUContext& ctx, memory::MMU& memory);

void rla_handler(CPUContext& ctx, memory::MMU& memory);

void rra_handler(CPUContext& ctx, memory::MMU& memory);

void stop_handler(CPUContext& ctx, memory::MMU& memory);

void daa_handler(CPUContext& ctx, memory::MMU& memory);

void cpl_handler(CPUContext& ctx, memory::MMU& memory);

void scf_handler(CPUContext& ctx, memory::MMU& memory);

void ccf_handler(CPUContext& ctx, memory::MMU& memory);

void halt_handler(CPUContext& ctx, memory::MMU& memory);

void di_handler(CPUContext& ctx, memory::MMU& memory);
void ei_handler(CPUContext& ctx, memory::MMU& memory);

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
    table_[std::to_underlying(InstructionType::STOP)] = stop_handler;

    table_[std::to_underlying(InstructionType::DI)] = di_handler;
    table_[std::to_underlying(InstructionType::EI)] = ei_handler;

    return table_;
}

};  // namespace gameboy::cpu