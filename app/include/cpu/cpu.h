#pragma once

#include <cstdint>
#include <stdexcept>

#include "cartridge/cartridge_header.h"
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

    CPUState state;
};

constexpr uint16_t reverse(uint16_t n)
{
    return ((n & 0xFF00) >> 8) | ((n & 0x00FF) << 8);
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
            auto instruction = fetch_instruction();
            auto& logger = logger::Logger::instance();
            logger.log(
                "PC: {:#x}, INST: {}, A: {:#x}, B:{:#x}, C:{:#x}", pc, get_instruction_name(instruction.type), regs.a,
                regs.b, regs.c);

            if (instruction.type == InstructionType::NONE) {
                throw std::runtime_error("Instruction is not valid or not yet added.");
            }
        }
    }

   private:
    auto fetch_instruction() noexcept -> const Instruction&
    {
        auto& registers = context_.registers;
        context_.current_opcode = memory_.read(registers.pc++);
        return instruction_set_[context_.current_opcode];
    }

    auto fetch_data(const Instruction& instruction)
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

    uint16_t cpu_read_reg(RegisterType rt)
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
};
}  // namespace gameboy::cpu
