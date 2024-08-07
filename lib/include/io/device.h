#pragma once

#include <cstdint>
#include "cpu/cpucontext.h"
#include "cpu/timer.h"
#include "io/gamepad.h"
#include "utils/logger.h"

namespace gameboy::io {

class Device
{
   public:
    Device() = delete;
    Device(gameboy::cpu::CPUContext& context) : context_(context)
    {
    }

    uint8_t read(uint16_t address) const noexcept;

    void write(uint16_t address, uint8_t value);

    auto gamepad() -> GamePad&
    {
        return gamepad_;
    }

   private:
    std::array<uint8_t, 2> serial_data_{};
    cpu::CPUContext& context_;

    GamePad gamepad_;
};
}  // namespace gameboy::io