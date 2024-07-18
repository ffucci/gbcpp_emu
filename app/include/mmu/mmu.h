#pragma once

#include <cstdint>
#include <stdexcept>
#include "cartridge/cartridge_header.h"
#include "mmu/ram.h"
namespace gameboy::memory {

// 0x0000 - 0x3FFF : ROM Bank 0
// 0x4000 - 0x7FFF : ROM Bank 1 - Switchable
// 0x8000 - 0x97FF : CHR RAM
// 0x9800 - 0x9BFF : BG Map 1
// 0x9C00 - 0x9FFF : BG Map 2
// 0xA000 - 0xBFFF : Cartridge RAM
// 0xC000 - 0xCFFF : RAM Bank 0
// 0xD000 - 0xDFFF : RAM Bank 1-7 - switchable - Color only
// 0xE000 - 0xFDFF : Reserved - Echo RAM
// 0xFE00 - 0xFE9F : Object Attribute Memory
// 0xFEA0 - 0xFEFF : Reserved - Unusable
// 0xFF00 - 0xFF7F : I/O Registers
// 0xFF80 - 0xFFFE : Zero Page

class MMU
{
   public:
    MMU() = delete;
    explicit MMU(cartridge::Cartridge& cartridge) : cartridge_(cartridge)
    {
    }

    auto read(uint16_t address) -> uint8_t
    {
        if (address < HRAM_LIMIT) {
            return cartridge_.read(address);
        } else if (address < CHR_RAM_LIMIT) {
            throw std::runtime_error("NOT IMPL");
        } else if (address < 0xC000) {
            return cartridge_.read(address);
        } else if (address < 0xE000) {
            // We go into the WRAM
            return ram_.wram_read(address);
        } else if (address < 0xFE00) {
            // Reserved echo RAM
            return 0;
        } else if (address < 0xFEA0) {
            throw std::runtime_error("NOT IMPL");
        } else if (address < 0xFF00) {
            // unused section
            return 0;
        } else if (address < 0xFF80) {
            throw std::runtime_error("IO");
        } else if (address == 0xFFFF) {
            throw std::runtime_error("CPU ENABLE REGISTER");
        }

        return ram_.hram_read(address);
    }

    auto read16(uint16_t address) -> uint16_t
    {
        auto low = read(address);
        auto hi = read(address + 1);
        return low | (hi << 8);
    }

    auto write(uint16_t address, uint8_t value) -> void
    {
        if (address < HRAM_LIMIT) {
            cartridge_.write(address, value);
            return;
        }

        if (address < CHR_RAM_LIMIT) {
            throw std::runtime_error("NOT IMPL");
        }

        if (address < 0xC000) {
            cartridge_.write(address, value);
            return;
        }

        if (address < 0xE000) {
            // WRAM write
            ram_.wram_write(address, value);
            return;
        }

        if (address < 0xFE00) {
            // Reserved echo RAM
            return;
        }

        if (address < 0xFEA0) {
            throw std::runtime_error("NOT IMPL");
        }

        if (address < 0xFF00) {
            // unused section
        }

        if (address < 0xFF80) {
            throw std::runtime_error("IO");
        }

        if (address == 0xFFFF) {
            throw std::runtime_error("CPU ENABLE REGISTER");
        }

        ram_.hram_write(address, value);
    }

    auto write16(uint16_t address, uint8_t value) -> void
    {
        write(address + 1, (value >> 8) & 0xFF);
        write(address, value & 0xFF);
    }

   private:
    cartridge::Cartridge& cartridge_;
    RAM ram_{};
    static constexpr uint16_t HRAM_LIMIT{0x8000};
    static constexpr uint16_t CHR_RAM_LIMIT{0xA000};
};

}  // namespace gameboy::memory
