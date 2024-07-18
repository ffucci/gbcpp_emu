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

static constexpr uint8_t ZERO = {1 << 7};
static constexpr uint8_t N_SUB = {1 << 6};
static constexpr uint8_t HALF = {1 << 5};
static constexpr uint8_t CARRY = {1 << 4};

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
        }
    }
};

using Handler = void (*)(CPUContext& context, memory::MMU&);
using ExecutorsTable = std::array<Handler, NUM_INSTRUCTION_TYPES>;

inline void none_handler(CPUContext& ctx, memory::MMU& memory)
{
    throw std::runtime_error("CANNOT EXECUTE");
}

inline void nop_handler(CPUContext& ctx, memory::MMU& memory)
{
}

inline void cpu_set_flag(uint8_t& flags, bool zero, bool n, bool half, bool carry)
{
    flags = (flags & ~ZERO) | (-zero & ZERO);
    flags = (flags & ~N_SUB) | (-n & N_SUB);
    flags = (flags & ~HALF) | (-half & HALF);
    flags = (flags & ~CARRY) | (-carry & CARRY);
}

inline void xor_handler(CPUContext& ctx, memory::MMU& memory)
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

inline void jp_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (check_cond(ctx)) {
        ctx.registers.pc = ctx.fetched_data;
        // emu_cycles(1)
    }
}

inline void ld_handler(CPUContext& ctx, memory::MMU& memory)
{
    if (ctx.destination_is_mem) {
        if (ctx.instruction.r2 == RegisterType::AF) {
            // emu_cycles(1)
            memory.write16(ctx.memory_destination, ctx.fetched_data);
        } else {
            memory.write(ctx.memory_destination, ctx.fetched_data);
        }
    }

    if (ctx.instruction.mode == AddressingMode::HL_SPR) {
        const auto reg2_val = ctx.cpu_read_reg(ctx.instruction.r2);

        bool hflag = (reg2_val & 0xF) + (ctx.fetched_data & 0xF) >= 0x10;
        bool cflag = (reg2_val & 0xFF) + (ctx.fetched_data & 0xFF) >= 0x100;

        cpu_set_flag(ctx.registers.f, false, false, hflag, cflag);
        ctx.cpu_set_reg(ctx.instruction.r1, reg2_val + (char)(ctx.fetched_data));  // TODO: understand better.
        return;
    }
    ctx.cpu_set_reg(ctx.instruction.r1, ctx.fetched_data);
}

inline void di_handler(CPUContext& ctx, memory::MMU& memory)
{
    ctx.interrupt_masked = true;
}

static constexpr auto make_executors_table() -> ExecutorsTable
{
    ExecutorsTable table_{};
    std::fill(begin(table_), end(table_), none_handler);
    table_[std::to_underlying(InstructionType::NOP)] = nop_handler;
    table_[std::to_underlying(InstructionType::XOR)] = xor_handler;
    table_[std::to_underlying(InstructionType::JP)] = jp_handler;
    table_[std::to_underlying(InstructionType::LD)] = ld_handler;
    table_[std::to_underlying(InstructionType::DI)] = di_handler;
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
                "{:#x}: {} ({:2X}, {:2X}, {:2X}), A: {:#x}, B:{:#x}, C:{:#x}, F:{:b}", pc,
                get_instruction_name(context_.instruction.type), context_.current_opcode, memory_.read(pc + 1),
                memory_.read(pc + 2), regs.a, regs.b, regs.c, regs.f);

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
            // emu_cycles(1)
            auto hi = memory_.read(regs.pc + 1);
            // emu_cycles(1)

            context_.fetched_data = (lo | (hi << 8));
            regs.pc += 2;
            return;
        };

        switch (instruction.mode) {
            case AddressingMode::IMP: {
                return;
            }
            case AddressingMode::R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r1);
                return;
            };
            case AddressingMode::R_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);
                return;
            }

            case AddressingMode::R_MR: {
                auto addr = context_.cpu_read_reg(instruction.r2);
                if (instruction.r1 == RegisterType::C) {
                    addr |= 0xFF00;
                }

                context_.fetched_data = memory_.read(addr);
                // emu_cycles(1);
                return;
            }
            // memory register to registers
            case AddressingMode::MR_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;  // understand how to optimize

                if (instruction.r1 == RegisterType::C) {
                    // If the memory destination is contained in register C then we need
                    // to set the higher bits to 1.
                    context_.memory_destination |= 0xFF00;
                }

                return;
            }
            case AddressingMode::R_D8: {
                rd8_handler();
                return;
            }
            case AddressingMode::R_HLI: {
                context_.fetched_data = memory_.read(context_.cpu_read_reg(instruction.r2));
                // emu_cycles(1)
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) + 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }
            case AddressingMode::R_HLD: {
                context_.fetched_data = memory_.read(context_.cpu_read_reg(instruction.r2));
                // emu_cycles(1)
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) - 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }

            case AddressingMode::HLI_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);  // What to increment read from registry
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) + 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }
            case AddressingMode::HLD_R: {
                context_.fetched_data = context_.cpu_read_reg(instruction.r2);  // What to increment read from registry
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                const auto new_val = context_.cpu_read_reg(RegisterType::HL) - 1;
                context_.cpu_set_reg(RegisterType::HL, new_val);
                return;
            }

            case AddressingMode::D8:
            case AddressingMode::R_A8: {
                context_.fetched_data = memory_.read(context_.registers.pc);
                // emu_cycles(1)
                context_.registers.pc++;
                return;
            }
            case AddressingMode::A8_R: {
                context_.memory_destination = memory_.read(context_.registers.pc) | 0xFF00;
                context_.destination_is_mem = true;
                // emu_cycles(1)

                context_.registers.pc++;
                return;
            }
            case AddressingMode::HL_SPR: {
                // load stack pointer
                context_.fetched_data = memory_.read(context_.registers.pc);
                // emu_cycles(1)
                context_.registers.pc++;
                return;
            }
            case AddressingMode::R_D16:
                [[fallthrough]];
            case AddressingMode::D16: {
                d_16_mode_handler();
                break;
            }

            case AddressingMode::A16_R:
                [[fallthrough]];
            case AddressingMode::D16_R: {
                auto& regs = context_.registers;

                auto lo = memory_.read(regs.pc);
                // emu_cycles(1)
                auto hi = memory_.read(regs.pc + 1);
                // emu_cycles(1)

                context_.memory_destination = (lo | (hi << 8));
                context_.destination_is_mem = true;
                regs.pc += 2;
                context_.fetched_data = memory_.read(context_.cpu_read_reg(instruction.r2));
                return;
            }
            case AddressingMode::MR_D8: {
                auto& regs = context_.registers;

                context_.fetched_data = memory_.read(regs.pc);
                // emu_cycles(1)
                regs.pc++;
                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                return;
            }
            case AddressingMode::MR: {
                auto& regs = context_.registers;

                context_.memory_destination = context_.cpu_read_reg(instruction.r1);
                context_.destination_is_mem = true;
                context_.fetched_data = memory_.read(context_.memory_destination);
                // emu_cycles(1)
                return;
            }
            case AddressingMode::R_A16: {
                auto& regs = context_.registers;

                auto lo = memory_.read(regs.pc);
                // emu_cycles(1)
                auto hi = memory_.read(regs.pc + 1);
                // emu_cycles(1)

                auto address = (lo | (hi << 8));
                regs.pc += 2;
                context_.fetched_data = memory_.read(address);
                // emu_cycles(1)
                return;
            }
            default:
                throw std::runtime_error("Unknown addressing mode");
        }
    }

    void execute(const Instruction& instruction)
    {
        // auto& logger = logger::Logger::instance();
        // logger.log("OPC: {:#x} , PC: {:#x} ", context_.current_opcode, context_.registers.pc);
        executors_[std::to_underlying(instruction.type)](context_, memory_);
    }

    CPUContext context_;
    memory::MMU memory_;
    static constexpr auto instruction_set_ = initialize_instruction_set();
    static constexpr auto executors_ = make_executors_table();
};
}  // namespace gameboy::cpu
