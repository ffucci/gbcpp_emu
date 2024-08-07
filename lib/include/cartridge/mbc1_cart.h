#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace gameboy::cartridge {

using RamBank = std::vector<uint8_t>;

class MBC1
{
   public:
    explicit MBC1(std::string filename, std::span<uint8_t> rom_base, uint8_t ram_size)
        : filename_(filename), rom_base_(std::move(rom_base))
    {
        setup_banks(ram_size);
    }

    auto read(uint16_t address) -> uint8_t
    {
        // A000 - BFFF
        if ((address & 0xE000) == 0xA000) {
            if (!ram_enabled) {
                return 0xFF;
            }

            if (!ram_bank) {
                return 0xFF;
            }

            return ram_bank[address - 0xA000];
        }

        return rom_bank_x[address - 0x4000];
    }

    void write(uint16_t address, uint8_t value)
    {
        uint16_t masked_address = (address & 0xE000);
        if (address < 0x2000) {
            ram_enabled = ((value & 0xF) == 0xA);
        }

        // ROM BANK
        if (masked_address == 0x2000) {
            value = (value == 0) ? 1 : value;

            value &= 0b11111;
            rom_bank_value = value;
            // Jump to the rom bank x
            rom_bank_x = rom_base_.data() + (ROM_BASE_ADDRESS * rom_bank_value);
        }

        // RAM BANK
        if (masked_address == 0x4000) {
            ram_bank_value = value & 0b11;

            if (ram_banking) {
                // if (need_save) {
                //     battery_save();
                // }
                ram_bank = ram_banks[ram_bank_value].data();
            }
        }

        // Range 0x6000
        if (masked_address == 0x6000) {
            banking_mode = value & 1;  // last bit
            ram_banking = value & 1;

            if (ram_banking) {
                // if (need_save) {
                //     BatterySaver<>::battery_save();
                // }
                ram_bank = ram_banks[ram_bank_value].data();
            }
        }

        // Region 4
        if (masked_address == 0xA000) {
            if (!ram_enabled) {
                return;
            }

            if (!ram_bank) {
                return;
            }

            ram_bank[address - 0xA000] = value;

            // if (battery) {
            //     need_save = true;
            // }
        }
    }

   private:
    void setup_banks(uint8_t ram_size)
    {
        for (uint8_t i = 0; i < NUMBER_RAM_BANKS; ++i) {
            // 2 means 1 bank

            bool check_for_ram = (ram_size == 2 && i == 0) || (ram_size == 3 && i < 4) || (ram_size == 4 && i < 16) ||
                                 (ram_size == 5 && i < 8);
            if (check_for_ram) {
                // std::cout << "Initializing RAM bank " << static_cast<int>(i) << std::endl;
                ram_banks[i].resize(RAM_BANK_SIZE, 0);
            }
        }

        ram_bank = ram_banks[0].data();
        rom_bank_x = rom_base_.data() + ROM_BASE_ADDRESS;  // rom bank 1
    }
    // MBC1 related data
    bool ram_enabled;
    bool ram_banking;

    std::span<uint8_t> rom_base_;
    uint8_t* rom_bank_x{nullptr};
    uint8_t banking_mode;

    uint8_t rom_bank_value;
    uint8_t ram_bank_value;

    uint8_t* ram_bank{nullptr};

    static constexpr uint8_t NUMBER_RAM_BANKS{16};
    static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};
    static constexpr uint16_t RAM_BANK_SIZE{0x2000};

    RamBank ram_banks[NUMBER_RAM_BANKS];  // number of banks

    std::string filename_;

    // battery
    bool battery;
    bool need_save{false};  // should save data
};
}  // namespace gameboy::cartridge