#include <cstdint>
#include "mmu/mmu.h"
#include "utils/logger.h"
#pragma

namespace gameboy::io {

class Device
{
   public:
    Device() = default;
    uint8_t read(uint16_t address) const noexcept
    {
        if (address == 0xFF01) {
            return serial_data_[0];
        }

        if (address == 0xFF02) {
            return serial_data_[1];
        }

        auto& logger = logger::Logger::instance();
        logger.log("Unsupported read access to {:04X}", address);
        return 0;
    }

    void write(uint16_t address, uint8_t value)
    {
        if (address == 0xFF01) {
            serial_data_[0] = value;
            return;
        }

        if (address == 0xFF02) {
            serial_data_[1] = value;
            return;
        }

        auto& logger = logger::Logger::instance();
        logger.log("Unsupported write access to {:04X}", address);
        return;
    }

   private:
    std::array<uint8_t, 2> serial_data_{};
};
}  // namespace gameboy::io