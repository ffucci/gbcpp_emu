#pragma once

#include <array>
#include <cstdint>

namespace gameboy::memory {

class RAM
{
   public:
    auto hram_read(uint16_t address) -> uint8_t
    {
        address -= HRAM_OFFSET;
        return hram_[address];
    }

    auto hram_write(uint16_t address, uint8_t value) -> void
    {
        address -= HRAM_OFFSET;
        hram_[address] = value;
    }

    auto wram_read(uint16_t address) -> uint8_t
    {
        address -= WRAM_OFFSET;
        return wram_[address];
    }

    auto wram_write(uint16_t address, uint8_t value) -> void
    {
        address -= WRAM_OFFSET;
        wram_[address] = value;
    }

   private:
    static constexpr uint16_t WRAM_OFFSET{0xC000};
    static constexpr uint16_t HRAM_OFFSET{0xFF80};

    std::array<uint8_t, 0x2000> wram_{};
    std::array<uint8_t, 0x890> hram_{};
};
}  // namespace gameboy::memory