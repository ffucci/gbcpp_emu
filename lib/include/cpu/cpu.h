#pragma once

#include <sys/types.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <format>
#include <stdexcept>
#include <thread>
#include <utility>

#include "cartridge/cartridge.h"
#include "cpu/cpucontext.h"
#include "cpu/instruction_handlers.h"

#include "cpu/interrupt.h"
#include "instructions.h"
#include "io/debug.h"
#include "mmu/dma.h"
#include "mmu/mmu.h"

namespace gameboy::cpu {

static constexpr std::string_view registers_name[] = {"<NONE>", "A",  "F",  "B",  "C",  "D",  "E", "H",
                                                      "L",      "AF", "BC", "DE", "HL", "SP", "PC"};

constexpr auto get_reg_name(RegisterType reg) -> std::string_view
{
    assert(index <= sizeof(registers_name));

    return registers_name[std::to_underlying(reg)];
}

constexpr std::string decode_instruction(const CPUContext& ctx, const memory::MMU& memory)
{
    auto inst = ctx.instruction;
    auto converted_instruction = std::format("{}", get_instruction_name(inst.type));

    switch (inst.mode) {
        case AddressingMode::IMP:
            break;

        case AddressingMode::R_D16:
        case AddressingMode::R_A16:
            converted_instruction =
                std::format("{} {},${:04X}", get_instruction_name(inst.type), get_reg_name(inst.r1), ctx.fetched_data);
            break;

        case AddressingMode::R:
            converted_instruction = std::format("{} {}", get_instruction_name(inst.type), get_reg_name(inst.r1));
            break;

        case AddressingMode::R_R:
            converted_instruction =
                std::format("{} {},{}", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::MR_R:
            converted_instruction = std::format(
                "{} ({}),{}", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::MR:
            converted_instruction = std::format("{} ({})", get_instruction_name(inst.type), get_reg_name(inst.r1));
            break;

        case AddressingMode::R_MR:
            converted_instruction = std::format(
                "{} {},({})", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::R_D8:
        case AddressingMode::R_A8:
            converted_instruction = std::format(
                "{} {},${:02X}", get_instruction_name(inst.type), get_reg_name(inst.r1), ctx.fetched_data & 0xFF);
            break;

        case AddressingMode::R_HLI:
            converted_instruction = std::format(
                "{} {},({}+)", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::R_HLD:
            converted_instruction = std::format(
                "{} {},({}-)", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::HLI_R:
            converted_instruction = std::format(
                "{} ({}+),{}", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::HLD_R:
            converted_instruction = std::format(
                "{} ({}-),{}", get_instruction_name(inst.type), get_reg_name(inst.r1), get_reg_name(inst.r2));
            break;

        case AddressingMode::A8_R:
            converted_instruction = std::format(
                "{} ${:02X},{}", get_instruction_name(inst.type), memory.read(ctx.registers.pc - 1),
                get_reg_name(inst.r2));
            break;

        case AddressingMode::HL_SPR:
            converted_instruction = std::format(
                "{} ({}),SP+{}", get_instruction_name(inst.type), memory.read(ctx.registers.pc - 1),
                ctx.fetched_data & 0xFF);
            break;

        case AddressingMode::D8:
            converted_instruction = std::format("{} ${:02X}", get_instruction_name(inst.type), ctx.fetched_data & 0xFF);
            break;

        case AddressingMode::D16:
            converted_instruction = std::format("{} ${:04X}", get_instruction_name(inst.type), ctx.fetched_data);
            break;

        case AddressingMode::MR_D8:
            converted_instruction = std::format(
                "{} ({}),${:02X}", get_instruction_name(inst.type), get_reg_name(inst.r1), ctx.fetched_data & 0xFF);
            break;

        case AddressingMode::A16_R:
            converted_instruction = std::format(
                "{} (${:04X}),{}", get_instruction_name(inst.type), ctx.fetched_data, get_reg_name(inst.r2));
            break;
        default:
            throw std::runtime_error("Invalid addressing mode...");
    }

    return converted_instruction;
}

class CPU
{
   public:
    CPU(CPURegisters initial_state, cartridge::Cartridge& cartridge)
        : context_{std::move(initial_state)}, memory_(cartridge, context_)
    {
        context_.registers.pc = 0x100;
        context_.registers.sp = 0xFFFE;
        *((short*)&context_.registers.a) = 0xB001;
        *((short*)&context_.registers.b) = 0x1300;
        *((short*)&context_.registers.d) = 0xD800;
        *((short*)&context_.registers.h) = 0x4D01;
        context_.interrupt_flags = 0;
        context_.master_interrupt_enabled = false;
        context_.enabling_ime = false;
    }

    void run(std::stop_token token);

    auto memory() -> const memory::MMU&
    {
        return memory_;
    }

   private:
    auto step() -> void;

    // Fetch and execute functions
    [[nodiscard]] auto fetch_instruction() noexcept -> const Instruction&;
    auto fetch_data(const Instruction& instruction) -> void;
    void execute(const Instruction& instruction);

    CPUContext context_{};
    memory::MMU memory_;
    InterruptHandler handler_{};
    static constexpr auto instruction_set_ = initialize_instruction_set();
    static constexpr auto executors_ = make_executors_table();

    io::Debug debug_{};

    static constexpr bool CPU_DEBUG{false};
};
}  // namespace gameboy::cpu
