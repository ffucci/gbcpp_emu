#pragma once

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class MBC5 final : public Cartridge
{
   public:
    explicit MBC5(std::string filename, RomBank rom_data, CartridgeMetadata metadata);

    [[nodiscard]] uint8_t read(uint16_t address) const override;

    void write(uint16_t address, uint8_t value) override;

   private:
    [[nodiscard]] auto selected_rom_bank() const noexcept -> uint16_t;

    [[nodiscard]] auto selected_ram_bank() const noexcept -> uint16_t;
};

}  // namespace gameboy::cartridge
