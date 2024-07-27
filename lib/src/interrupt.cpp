#include "cpu/interrupt.h"

namespace gameboy::cpu {

void InterruptHandler::handle_interrupts(CPUContext& context, memory::MMU& memory)
{
    if (check_for_interrupt(context, memory, 0x40, InterruptType::VBLANK)) {
        return;
    }

    if (check_for_interrupt(context, memory, 0x48, InterruptType::LCD_STAT)) {
        return;
    }

    if (check_for_interrupt(context, memory, 0x50, InterruptType::TIMER)) {
        return;
    }

    if (check_for_interrupt(context, memory, 0x58, InterruptType::SERIAL)) {
        return;
    }

    if (check_for_interrupt(context, memory, 0x60, InterruptType::JOYPAD)) {
        return;
    }
}
bool InterruptHandler::check_for_interrupt(
    CPUContext& context, memory::MMU& memory, uint16_t address, InterruptType interrupt)
{
    auto to_byte_interrupt = std::to_underlying(interrupt);
    if ((context.interrupt_flags & to_byte_interrupt) && (memory.ie_register() & to_byte_interrupt)) {
        handle(context, memory, address);
        context.interrupt_flags &= ~to_byte_interrupt;
        context.state = CPUState::RUNNING;
        context.master_interrupt_enabled = false;
        return true;
    }

    return false;
}
void InterruptHandler::handle(CPUContext& context, memory::MMU& memory, uint16_t address)
{
    // NEEDS TO SAVE THE OLD PROGRAM COUNTER AND JUMP TO THE INTERRUPT HANDLER
    stack_push16(context, memory, context.registers.pc);
    context.registers.pc = address;
}
}  // namespace gameboy::cpu