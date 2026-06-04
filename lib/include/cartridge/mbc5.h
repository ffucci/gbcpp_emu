#pragma once

#include <algorithm>

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class MBC5 : public Cartridge
{
   public:
    explicit MBC5(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
    }

    uint8_t read(uint16_t address) const override
    {
        if (address < ROM_BASE_ADDRESS) {
            return rom_data_[address];
        }

        if (address_in_range(address, 0x4000, 0x8000)) {
            const uint16_t addr_within_bank = address - ROM_BASE_ADDRESS;
            const uint32_t bank_offset = ROM_BASE_ADDRESS * selected_rom_bank();
            return rom_data_[bank_offset + addr_within_bank];
        }

        if (address_in_range(address, 0xA000, 0xC000)) {
            if (!ram_enabled_ || ram_data_.empty()) {
                return 0xFF;
            }

            const uint16_t addr_within_bank = address - 0xA000;
            const uint32_t bank_offset = RAM_BANK_SIZE * selected_ram_bank();
            return ram_data_[bank_offset + addr_within_bank];
        }

        return 0xFF;
    }

    void write(uint16_t address, uint8_t value) override
    {
        if (address < 0x2000) {
            ram_enabled_ = ((value & 0x0F) == 0x0A);
            return;
        }

        if (address_in_range(address, 0x2000, 0x3000)) {
            rom_idx_ = (rom_idx_ & 0x100) | value;
            return;
        }

        if (address_in_range(address, 0x3000, 0x4000)) {
            rom_idx_ = (rom_idx_ & 0x0FF) | ((value & 0x01) << 8);
            return;
        }

        if (address_in_range(address, 0x4000, 0x6000)) {
            ram_idx_ = value & 0x0F;
            return;
        }

        if (address_in_range(address, 0xA000, 0xC000)) {
            if (!ram_enabled_ || ram_data_.empty()) {
                return;
            }

            const uint16_t addr_within_bank = address - 0xA000;
            const uint32_t bank_offset = RAM_BANK_SIZE * selected_ram_bank();
            ram_data_[bank_offset + addr_within_bank] = value;

            if (battery_saver) {
                battery_saver->save(filename_, ram_data_.data() + bank_offset);
            }
        }
    }

   private:
    [[nodiscard]] auto selected_rom_bank() const noexcept -> uint16_t
    {
        const uint16_t num_banks = std::max<uint16_t>(1, rom_data_.size() / ROM_BASE_ADDRESS);
        return rom_idx_ % num_banks;
    }

    [[nodiscard]] auto selected_ram_bank() const noexcept -> uint16_t
    {
        const uint16_t num_banks = std::max<uint16_t>(1, ram_data_.size() / RAM_BANK_SIZE);
        return ram_idx_ % num_banks;
    }
};

}  // namespace gameboy::cartridge
