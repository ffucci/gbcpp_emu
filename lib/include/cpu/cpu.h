#pragma once

#include <sys/types.h>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <thread>
#include <utility>

#include "cartridge/cartridge.h"
#include "cpu/cpucontext.h"
#include "cpu/instruction_handlers.h"

#include "instructions.h"
#include "mmu/mmu.h"

namespace gameboy::cpu {

class CPU
{
   public:
    CPU(CPURegisters initial_state, cartridge::Cartridge& cartridge)
        : context_{.registers = std::move(initial_state)}, memory_(cartridge)
    {
    }

    void run();

   private:
    // Fetch and execute functions
    [[nodiscard]] auto fetch_instruction() noexcept -> const Instruction&;
    auto fetch_data(const Instruction& instruction) -> void;
    void execute(const Instruction& instruction);

    CPUContext context_;
    memory::MMU memory_;

    size_t ticks_{0};
    static constexpr auto instruction_set_ = initialize_instruction_set();
    static constexpr auto executors_ = make_executors_table();
};
}  // namespace gameboy::cpu
