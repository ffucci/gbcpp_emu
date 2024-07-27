#include "io/device.h"

namespace gameboy::io {

uint8_t Device::read(uint16_t address) const noexcept
{
    if (address == 0xFF01) {
        return serial_data_[0];
    }

    if (address == 0xFF02) {
        return serial_data_[1];
    }

    if ((address >= 0xFF04) && (address <= 0xFF07)) {
        return context_.timer.read(address);
    }

    if (address == 0xFF0F) {
        return context_.interrupt_flags;
    }

    auto& logger = logger::Logger::instance();
    logger.log("UNSUPPORTED bus_read({:04X})", address);
    return 0;
}

void Device::write(uint16_t address, uint8_t value)
{
    auto& logger = logger::Logger::instance();

    if (address == 0xFF01) {
        serial_data_[0] = value;
        return;
    }

    if (address == 0xFF02) {
        serial_data_[1] = value;
        return;
    }

    if ((address >= 0xFF04) && (address <= 0xFF07)) {
        logger.log("TIMER_WRITE {:04X}", address);
        context_.timer.write(address, value);
        return;
    }

    if (address == 0xFF0F) {
        context_.interrupt_flags = value;
        return;
    }

    logger.log("Unsupported write access to {:04X}", address);
    return;
}
}  // namespace gameboy::io