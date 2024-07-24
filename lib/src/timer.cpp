#include "cpu/timer.h"
#include <stdexcept>

#include "utils/logger.h"

namespace gameboy::cpu::timer {

void Timer::tick(uint8_t& interrupt_flags)
{
    auto& logger = logger::Logger::instance();

    uint16_t prev_div = context_.div;
    context_.div++;

    bool timer_update = false;
    timer_update =
        (prev_div & (1 << bitmap_[context_.tac & 0b11])) && (!(context_.div & (1 << bitmap_[context_.tac & 0b11])));

    // logger.log("div: {} timer_update: {}, tac: {}", context_.div, timer_update, context_.tac);

    if (timer_update && context_.tac & (1 << 2)) {
        context_.tima++;
        logger.log("TIMA: {}", context_.tima);

        if (context_.tima == 0xFF) {
            context_.tima = context_.tma;
            interrupt_flags |= std::to_underlying(InterruptType::TIMER);
            logger.log("INTERRUPT: {}", std::to_underlying(InterruptType::TIMER));
        }
    }
}

void Timer::write(uint16_t address, uint8_t value)
{
    switch (address) {
        case 0xFF04:
            // DIV
            context_.div = 0;
            break;
        case 0xFF05:
            // DIV
            context_.tima = value;
            break;
        case 0xFF06:
            // DIV
            context_.tma = value;
            break;
        case 0xFF07:
            // TAC
            std::cout << "TAC: " << static_cast<int>(context_.tac) << std::endl;
            context_.tac = value;
            break;
    }
}
auto Timer::read(uint16_t address) const -> uint8_t
{
    switch (address) {
        case 0xFF04:
            return context_.div >> 8;
        case 0xFF05:
            return context_.tima;
        case 0xFF06:
            return context_.tma;
        case 0xFF07:
            return context_.tac;
    }

    throw std::runtime_error("NEVER HERE");
}
}  // namespace gameboy::cpu::timer