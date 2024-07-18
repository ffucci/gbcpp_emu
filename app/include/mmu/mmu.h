#pragma once

#include <cstdint>
#include <stdexcept>
#include "cartridge/cartridge_header.h"
namespace gameboy::memory {

class MMU
{
   public:
    MMU() = delete;
    explicit MMU(cartridge::Cartridge& cartridge) : cartridge_(cartridge)
    {
    }

    auto read(uint16_t address) -> uint8_t
    {
        if (address < 0x8000) {
            return cartridge_.read(address);
        }

        throw std::runtime_error("NOT IMPL");
    }

    auto read16(uint16_t address) -> uint16_t
    {
        auto low = read(address);
        auto hi = read(address + 1);
        return low | (hi << 8);
    }

    auto write(uint16_t address, uint8_t value) -> void
    {
        if (address < 0x8000) {
            cartridge_.write(address, value);
            return;
        }

        // throw std::runtime_error("NOT IMPL");
    }

    auto write16(uint16_t address, uint8_t value) -> void
    {
        write(address + 1, (value >> 8) & 0xFF);
        write(address, value & 0xFF);
    }

   private:
    cartridge::Cartridge& cartridge_;
};

}  // namespace gameboy::memory
