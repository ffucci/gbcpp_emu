#pragma once

#include <cstdint>
#include <utility>
#include "cpu/cpucontext.h"
#include "cpu/instruction_handlers.h"
#include "cpu/instructions.h"
#include "mmu/mmu.h"
namespace gameboy::cpu {

enum class InterruptType
{
    VBLANK = 1,
    LCD_STAT = 2,
    TIMER = 4,
    SERIAL = 8,
    JOYPAD = 16
};

class InterruptHandler
{
   public:
    InterruptHandler() = default;

    void handle_interrupts(CPUContext& context, memory::MMU& memory);

   private:
    bool check_for_interrupt(CPUContext& context, memory::MMU memory, uint16_t address, InterruptType interrupt);
    void handle(CPUContext& context, memory::MMU memory, uint16_t address);
};

}  // namespace gameboy::cpu