#include <cstdint>
#include <stop_token>

#include "cpu/cpu.h"
#include "cpu/cpucontext.h"
#include "cpu/instructions.h"
#include "cpu/utils.h"
#include "utils/logger.h"

namespace gameboy::cpu {

auto CPU::fetch_data(const Instruction& instruction) -> void
{
    context_.memory_destination = 0;
    context_.destination_is_mem = false;

    const auto rd8_handler = [this]() {
        auto& regs = context_.registers;
        context_.fetched_data = memory_.read(regs.pc);
        update_cycles(1, context_, memory_);
        regs.pc++;
        return;
    };

    const auto d_16_mode_handler = [this]() {
        auto& regs = context_.registers;

        auto lo = memory_.read(regs.pc);
        update_cycles(1, context_, memory_);
        auto hi = memory_.read(regs.pc + 1);
        update_cycles(1, context_, memory_);

        context_.fetched_data = (lo | (hi << 8));
        regs.pc += 2;
        return;
    };

    switch (instruction.mode) {
        case AddressingMode::IMP: {
            return;
        }
        case AddressingMode::R: {
            context_.fetched_data = context_.read_reg(instruction.r1);
            return;
        };
        case AddressingMode::R_R: {
            context_.fetched_data = context_.read_reg(instruction.r2);
            return;
        }
        case AddressingMode::R_D8: {
            rd8_handler();
            return;
        }
        case AddressingMode::R_MR: {
            auto addr = context_.read_reg(instruction.r2);
            if (instruction.r2 == RegisterType::C) {
                addr |= 0xFF00;
            }

            context_.fetched_data = memory_.read(addr);
            update_cycles(1, context_, memory_);
            return;
        }
        // memory register to registers
        case AddressingMode::MR_R: {
            context_.fetched_data = context_.read_reg(instruction.r2);
            context_.memory_destination = context_.read_reg(instruction.r1);
            context_.destination_is_mem = true;  // understand how to optimize

            if (instruction.r1 == RegisterType::C) {
                // If the memory destination is contained in register C then we need
                // to set the higher bits to 1.
                context_.memory_destination |= 0xFF00;
            }

            return;
        }
        case AddressingMode::R_HLI: {
            context_.fetched_data = memory_.read(context_.read_reg(instruction.r2));
            update_cycles(1, context_, memory_);
            const uint16_t new_val = context_.read_reg(RegisterType::HL) + 1;
            context_.set_reg(RegisterType::HL, new_val);
            return;
        }
        case AddressingMode::R_HLD: {
            context_.fetched_data = memory_.read(context_.read_reg(instruction.r2));
            update_cycles(1, context_, memory_);
            const uint16_t new_val = context_.read_reg(RegisterType::HL) - 1;
            context_.set_reg(RegisterType::HL, new_val);
            return;
        }
        case AddressingMode::HLI_R: {
            context_.fetched_data = context_.read_reg(instruction.r2);  // What to increment read from registry
            context_.memory_destination = context_.read_reg(instruction.r1);
            context_.destination_is_mem = true;
            const auto new_val = context_.read_reg(RegisterType::HL) + 1;
            context_.set_reg(RegisterType::HL, new_val);
            return;
        }
        case AddressingMode::HLD_R: {
            context_.fetched_data = context_.read_reg(instruction.r2);  // What to increment read from registry
            context_.memory_destination = context_.read_reg(instruction.r1);
            context_.destination_is_mem = true;
            const auto new_val = context_.read_reg(RegisterType::HL) - 1;
            context_.set_reg(RegisterType::HL, new_val);
            return;
        }

        case AddressingMode::D8:
        case AddressingMode::R_A8: {
            context_.fetched_data = memory_.read(context_.registers.pc);
            update_cycles(1, context_, memory_);
            context_.registers.pc++;
            return;
        }
        case AddressingMode::A8_R: {
            context_.memory_destination = memory_.read(context_.registers.pc) | 0xFF00;
            context_.destination_is_mem = true;
            update_cycles(1, context_, memory_);
            context_.registers.pc++;
            return;
        }
        case AddressingMode::HL_SPR: {
            // load stack pointer
            context_.fetched_data = memory_.read(context_.registers.pc);
            update_cycles(1, context_, memory_);
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

            uint16_t lo = memory_.read(regs.pc);
            update_cycles(1, context_, memory_);
            uint16_t hi = memory_.read(regs.pc + 1);
            update_cycles(1, context_, memory_);

            context_.memory_destination = (lo | (hi << 8));
            context_.destination_is_mem = true;
            regs.pc += 2;
            context_.fetched_data = context_.read_reg(instruction.r2);
            return;
        }
        case AddressingMode::MR_D8: {
            auto& regs = context_.registers;

            context_.fetched_data = memory_.read(regs.pc);
            update_cycles(1, context_, memory_);
            regs.pc++;
            context_.memory_destination = context_.read_reg(instruction.r1);
            context_.destination_is_mem = true;
            return;
        }
        case AddressingMode::MR: {
            auto& regs = context_.registers;

            context_.memory_destination = context_.read_reg(instruction.r1);
            context_.destination_is_mem = true;
            context_.fetched_data = memory_.read(context_.memory_destination);
            update_cycles(1, context_, memory_);
            return;
        }
        case AddressingMode::R_A16: {
            auto& regs = context_.registers;

            uint16_t lo = memory_.read(regs.pc);
            update_cycles(1, context_, memory_);
            uint16_t hi = memory_.read(regs.pc + 1);
            update_cycles(1, context_, memory_);

            uint16_t address = lo | (hi << 8);
            regs.pc += 2;

            context_.fetched_data = memory_.read(address);
            update_cycles(1, context_, memory_);
            return;
        }
        default:
            throw std::runtime_error("Unknown addressing mode");
    }
}
void CPU::execute(const Instruction& instruction)
{
    executors_[std::to_underlying(instruction.type)](context_, memory_);
}
auto CPU::fetch_instruction() noexcept -> const Instruction&
{
    auto& registers = context_.registers;
    context_.current_opcode = memory_.read(registers.pc++);
    return instruction_set_[context_.current_opcode];
}
void CPU::run(std::stop_token token)
{
    context_.state = CPUState::RUNNING;

    while (!token.stop_requested()) {
        if (context_.state == CPUState::RUNNING) {
            step();
        } else {
            update_cycles(1, context_, memory_);
            if (context_.interrupt_flags) {
                context_.state = CPUState::RUNNING;
            }
        }

        if (context_.master_interrupt_enabled) {
            handler_.handle_interrupts(context_, memory_);
            context_.enabling_ime = false;
        }

        if (context_.enabling_ime) {
            context_.master_interrupt_enabled = true;
        }

        update_cycles(1, context_, memory_);
    }
}

void CPU::step()
{
    auto& regs = context_.registers;

    auto pc = regs.pc;
    context_.instruction = fetch_instruction();
    update_cycles(1, context_, memory_);
    fetch_data(context_.instruction);

    if constexpr (CPU_DEBUG) {
        auto& logger = logger::Logger::instance();
        logger.log(
            "{:08X} - {:04X}: {:12} ({:02X} {:02X} {:02X}) A: {:02X} F: {} BC: {:02X}{:02X} DE: {:02X}{:02X} "
            "HL: {:02X}{:02X}",
            context_.ticks, pc, decode_instruction(context_, memory_), context_.current_opcode, memory_.read(pc + 1),
            memory_.read(pc + 2), regs.a, regs.get_flags_string(), regs.b, regs.c, regs.d, regs.e, regs.h, regs.l);
    }

    if (context_.instruction.type == InstructionType::NONE) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        throw std::runtime_error("Instruction is not valid or not yet added.");
    }

    debug_.update(memory_);
    execute(context_.instruction);
}

}  // namespace gameboy::cpu