#pragma once

#include <sys/types.h>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "cartridge/cartridge_header.h"
#include "cpu/executor.h"
#include "instructions.h"
#include "mmu/mmu.h"
#include "utils/logger.h"

namespace gameboy::cpu {

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
};

inline auto get_initial_state() -> CPURegisters
{
    // DMG
    return {0x1, 0, 0x0, 0x13, 0x00, 0xD8, 0x01, 0x4d, 0x100, 0xFFFE};
}

enum class CPUState
{
    HALT,
    RUNNING
};

struct CPUContext
{
    CPURegisters registers;
    uint16_t fetched_data;
    uint16_t memory_destination;
    uint8_t current_opcode;

    Instruction instruction;
    CPUState state;
};

constexpr uint16_t reverse(uint16_t n)
{
    return ((n & 0xFF00) >> 8) | ((n & 0x00FF) << 8);
}

using Handler = void (*)(CPUContext& context);
using ExecutorsTable = std::array<Handler, NUM_INSTRUCTION_TYPES>;

inline void none_handler(CPUContext& ctx)
{
    throw std::runtime_error("CANNOT EXECUTE");
}

inline void nop_handler(CPUContext& ctx)
{
}

static constexpr uint8_t ZERO = {1 << 7};
static constexpr uint8_t N_SUB = {1 << 6};
static constexpr uint8_t HALF = {1 << 5};
static constexpr uint8_t CARRY = {1 << 4};

inline void cpu_set_flag(uint8_t& flags, bool zero, bool n, bool half, bool carry)
{
    flags = (flags & ~ZERO) | (-zero & ZERO);
    flags = (flags & ~N_SUB) | (-n & N_SUB);
    flags = (flags & ~HALF) | (-half & HALF);
    flags = (flags & ~CARRY) | (-carry & CARRY);
}

inline void xor_handler(CPUContext& ctx)
{
    auto& regs = ctx.registers;
    regs.a ^= ctx.fetched_data;
    cpu_set_flag(regs.f, regs.a == 0, false, false, false);
}

inline bool check_cond(CPUContext& context)
{
    bool z = (context.registers.f & ZERO) != 0;
    bool c = (context.registers.f & CARRY) != 0;

    switch (context.instruction.condition) {
        case ConditionType::None:
            return true;
        case ConditionType::C:
            return c;
        case ConditionType::NC:
            return !c;
        case ConditionType::Z:
            return z;
        case ConditionType::NZ:
            return !z;
    }

    return false;
}

inline void jp_handler(CPUContext& ctx)
{
    if (check_cond(ctx)) {
        ctx.registers.pc = ctx.fetched_data;
        // emulate cycles
    }
}

static constexpr auto make_executors_table() -> ExecutorsTable
{
    ExecutorsTable table_{};
    std::fill(begin(table_), end(table_), none_handler);
    table_[std::to_underlying(InstructionType::NOP)] = nop_handler;
    table_[std::to_underlying(InstructionType::XOR)] = xor_handler;
    table_[std::to_underlying(InstructionType::JP)] = jp_handler;
    return table_;
}

class CPU
{
   public:
    CPU(CPURegisters initial_state, cartridge::Cartridge& cartridge)
        : context_{.registers = std::move(initial_state)}, memory_(cartridge)
    {
    }

    void run()
    {
        context_.state = CPUState::RUNNING;
        auto& regs = context_.registers;

        while (context_.state != CPUState::HALT) {
            auto pc = regs.pc;
            context_.instruction = fetch_instruction();
            fetch_data(context_.instruction);
            auto& logger = logger::Logger::instance();
            logger.log(
                "PC: {:#x}, INST: ({}, {:#x}), A: {:#x}, B:{:#x}, C:{:#x}, F:{:b}", pc,
                get_instruction_name(context_.instruction.type), context_.current_opcode, regs.a, regs.b, regs.c,
                regs.f);

            if (context_.instruction.type == InstructionType::NONE) {
                throw std::runtime_error("Instruction is not valid or not yet added.");
            }

            execute(context_.instruction);
        }
    }

   private:
    auto fetch_instruction() noexcept -> const Instruction&
    {
        auto& registers = context_.registers;
        context_.current_opcode = memory_.read(registers.pc++);
        return instruction_set_[context_.current_opcode];
    }

    auto fetch_data(const Instruction& instruction) -> void
    {
        const auto rd8_handler = [this]() {
            auto& regs = context_.registers;
            context_.fetched_data = memory_.read(regs.pc);
            regs.pc++;
            return;
        };

        const auto d_16_mode_handler = [this]() {
            auto& regs = context_.registers;

            auto lo = memory_.read(regs.pc);
            // emu_cycles
            auto hi = memory_.read(regs.pc + 1);
            // emu_cycles

            context_.fetched_data = (lo | (hi << 8));
            regs.pc += 2;
            return;
        };

        switch (instruction.mode) {
            case AddressingMode::IMP: {
                return;
            }
            // case AddressingMode::R_D16:
            // case AddressingMode::R_R:
            // case AddressingMode::MR_R:
            case AddressingMode::R: {
                context_.fetched_data = cpu_read_reg(instruction.r1);
                return;
            };
            case AddressingMode::R_D8: {
                rd8_handler();
            }
            // case AddressingMode::R_MR:
            // case AddressingMode::R_HLI:
            // case AddressingMode::R_HLD:
            // case AddressingMode::HLI_R:
            // case AddressingMode::HLD_R:
            // case AddressingMode::R_A8:
            // case AddressingMode::A8_R:
            // case AddressingMode::HL_SPR:
            case AddressingMode::D16: {
                d_16_mode_handler();
            }
            // case AddressingMode::D8:
            // case AddressingMode::D16_R:
            // case AddressingMode::MR_D8:
            // case AddressingMode::MR:
            case AddressingMode::A16_R: {
                return;
            }
            // case AddressingMode::R_A16:
            //     break;
            default:
                throw std::runtime_error("Unknown addressing mode");
        }
    }

    void execute(const Instruction& instruction)
    {
        auto& logger = logger::Logger::instance();
        logger.log("OPC: {:#x} , PC: {:#x} ", context_.current_opcode, context_.registers.pc);
        executors_[std::to_underlying(instruction.type)](context_);
    }

    auto cpu_read_reg(RegisterType rt) -> uint16_t
    {
        auto& regs = context_.registers;
        using gameboy::cpu::RegisterType;
        switch (rt) {
            case RegisterType::A:
                return regs.a;
            case RegisterType::F:
                return regs.f;
            case RegisterType::B:
                return regs.b;
            case RegisterType::C:
                return regs.c;
            case RegisterType::D:
                return regs.d;
            case RegisterType::E:
                return regs.e;
            case RegisterType::H:
                return regs.h;
            case RegisterType::L:
                return regs.l;

            case RegisterType::AF:
                return reverse(*((uint16_t*)&regs.a));
            case RegisterType::BC:
                return reverse(*((uint16_t*)&regs.b));
            case RegisterType::DE:
                return reverse(*((uint16_t*)&regs.d));
            case RegisterType::HL:
                return reverse(*((uint16_t*)&regs.h));
            case RegisterType::PC:
                return regs.pc;
            case RegisterType::SP:
                return regs.sp;
            default:
                return 0;
        }
    }

    CPUContext context_;
    memory::MMU memory_;
    static constexpr auto instruction_set_ = initialize_instruction_set();
    static constexpr auto executors_ = make_executors_table();
};
}  // namespace gameboy::cpu
