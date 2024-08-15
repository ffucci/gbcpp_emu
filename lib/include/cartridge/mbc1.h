#pragma once

#include "cartridge/cartridge.h"

namespace gameboy::cartridge {

class MBC1 : public Cartridge
{
   public:
    explicit MBC1(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
        rom_idx_ = 1;
        std::cout << "Size of ram: " << ram_data_.size() << std::endl;
    }

    uint8_t read(uint16_t address) const override
    {
        if (address < ROM_BASE_ADDRESS) {
            return rom_data_[address];
        }

        uint16_t masked_address = (address & 0xE000);  // 1110 0000

        // The last space is reserved to RAM
        if (masked_address == 0xA000) {
            if (!ram_enabled_) {
                return 0xFF;
            }
            uint16_t addr_within_bank = address - 0xA000;
            uint16_t bank_offset = 0x2000 * ram_idx_;
            return ram_data_[bank_offset + addr_within_bank];
        }

        uint16_t addr_within_bank = address - ROM_BASE_ADDRESS;
        uint32_t bank_offset = ROM_BASE_ADDRESS * rom_idx_;
        return rom_data_[bank_offset + addr_within_bank];
    }

    void write(uint16_t address, uint8_t value) override
    {
        uint16_t masked_address = (address & 0xE000);  // 1110 0000
        if (address < 0x2000) {
            ram_enabled_ = ((value & 0xF) == 0xA);
        }

        if (masked_address == 0x2000) {
            value = (value == 0) ? 1 : value;
            value &= 0b11111;
            rom_idx_ = value;
            return;
        }

        if (masked_address == 0x4000) {
            value &= 0b11;

            if (need_save_) {
                save_to_battery();
            }

            ram_idx_ = value;
            return;
        }

        // The last space is reserved to RAM
        if (masked_address == 0x6000) {
            ram_banking = value & 1;

            if (ram_banking) {
                save_to_battery();
            }
        }

        // The last space is reserved to RAM
        if (masked_address == 0xA000) {
            if (!ram_enabled_) {
                return;
            }
            uint16_t addr_within_bank = address - 0xA000;
            uint32_t bank_offset = 0x2000 * ram_idx_;
            ram_data_[bank_offset + addr_within_bank] = value;

            if (header_.has_battery()) {
                need_save_ = true;
            }
        }
    }

    ~MBC1()
    {
        save_to_battery();
    }

   private:
    void save_to_battery()
    {
        if (battery_saver) {
            uint32_t bank_offset = 0x2000 * ram_idx_;
            auto ram_bank = ram_data_.data() + bank_offset;
            battery_saver->save(filename_, ram_bank);
        }
    }
    uint8_t ram_banking{0x0};
};
}  // namespace gameboy::cartridge