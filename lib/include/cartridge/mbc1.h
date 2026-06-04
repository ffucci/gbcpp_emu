#pragma once

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class MBC1 : public Cartridge
{
   public:
    explicit MBC1(std::string filename, RomBank rom_data, CartridgeMetadata metadata);

    uint8_t read(uint16_t address) const override;

    void write(uint16_t address, uint8_t value) override;

    ~MBC1() override;

   private:
    void save_to_battery();

    uint8_t ram_banking{0x0};
};
}  // namespace gameboy::cartridge
