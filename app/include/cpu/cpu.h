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
        while (context_.state != CPUState::HALT) {
            auto pc = context_.registers.pc;
            auto instruction = fetch_instruction();

            auto& logger = logger::Logger::instance();
            logger.log("PC: {:#x}, INST: {}", pc, get_instruction_name(instruction.type));

            if (instruction.type == InstructionType::NONE) {
                throw std::runtime_error("Instruction is not valid or not yet added.");
            }
        }
    }

   private:
    void next() noexcept
    {
    }

    auto fetch_instruction() noexcept -> const Instruction&
    {
        auto& registers = context_.registers;
        context_.current_opcode = memory_.read(registers.pc++);
        return instruction_set_[context_.current_opcode];
    }

    CPUContext context_;
    memory::MMU memory_;
    static constexpr auto instruction_set_ = initialize_instruction_set();
};
}  // namespace gameboy::cpu
