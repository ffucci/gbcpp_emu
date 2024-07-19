#include "cpu/cpu.h"

namespace gameboy::cpu {

auto CPU::fetch_data(const Instruction& instruction) -> void
{
    context_.memory_destination = 0;
    context_.destination_is_mem = false;

    const auto rd8_handler = [this]() {
        auto& regs = context_.registers;
        context_.fetched_data = memory_.read(regs.pc);
        // auto& logger = logger::Logger::instance();
        // logger.log("Fetched data... {:02X}", context_.fetched_data);
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
void CPU::execute(const Instruction& instruction)
{
    if constexpr (DEBUG) {
        auto& logger = logger::Logger::instance();
        logger.log("OPC: {:#x} , PC: {:#x} ", context_.current_opcode, context_.registers.pc);
    }
    executors_[std::to_underlying(instruction.type)](context_, memory_);
}
auto CPU::fetch_instruction() noexcept -> const Instruction&
{
    auto& registers = context_.registers;
    context_.current_opcode = memory_.read(registers.pc++);
    return instruction_set_[context_.current_opcode];
}
void CPU::run()
{
    context_.state = CPUState::RUNNING;
    auto& regs = context_.registers;

    while (context_.state != CPUState::HALT) {
        auto pc = regs.pc;
        context_.instruction = fetch_instruction();
        fetch_data(context_.instruction);
        auto& logger = logger::Logger::instance();

        logger.log(
            "{:08X} - {:04X}: {} ({:02X}, {:02X}, {:02X}) \tA: {:02X}, BC:{:02X}{:02X}, DE:{:02X}{:02X}, "
            "HL:{:02X}{:02X}, "
            "F:{}",
            ticks_, pc, get_instruction_name(context_.instruction.type), context_.current_opcode, memory_.read(pc + 1),
            memory_.read(pc + 2), regs.a, regs.b, regs.c, regs.d, regs.e, regs.h, regs.l, regs.get_flags_string());

        if (context_.instruction.type == InstructionType::NONE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            throw std::runtime_error("Instruction is not valid or not yet added.");
        }

        execute(context_.instruction);
        ticks_++;
    }
}

}  // namespace gameboy::cpu