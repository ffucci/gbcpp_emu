#pragma once

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class NoMBC : public Cartridge
{
   public:
    explicit NoMBC(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
    }

    uint8_t read(uint16_t address) const override
    {
        return rom_data_[address];
    }

    void write(uint16_t address, uint8_t) override
    {
    }
};

}  // namespace gameboy::cartridge