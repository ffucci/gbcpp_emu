#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <span>
#include <utility>

#include "utils/logger.h"
#include "utils/utils.h"

namespace gameboy::cartridge {

// Add the cartridge type
enum class CartridgeType : uint8_t
{
    RomOnly = 0,
    MBC1,
    MBC1_RAM,
    MBC1_RAM_BATTERY,
    MBC2,
    MBC2_BATTERY,
    // TODO: Add all of them (MBC1 for now is enough)
    MBC3,
    MBC4,
    MBC5,
    Unknown
};

using RamBank = std::vector<uint8_t>;

struct RomHeader
{
    static constexpr uint8_t NUM_ENTRY_BITS{4};
    static constexpr uint8_t NUM_LOGO_BITS{0x30};
    static constexpr uint32_t ENTRY_POINT{0x100};

    uint8_t entry[NUM_ENTRY_BITS];
    uint8_t logo[NUM_LOGO_BITS];
    std::array<char, 16> title{};
    uint16_t new_license_code;
    uint8_t sgb_flag;
    CartridgeType cartridge_type;
    uint8_t rom_size;
    uint8_t ram_size;
    uint8_t destination_code;
    uint8_t old_license_code;
    uint8_t version;
    uint8_t checksum;
    uint16_t global_checksum;
};

inline auto compute_checksum(std::span<uint8_t> rom_view) -> uint8_t
{
    uint8_t checksum = 0;
    for (uint16_t address = 0x0134; address <= 0x014C; address++) {
        checksum = checksum - rom_view[address] - 1;
    }

    return checksum;
}
inline RomHeader from_rom(std::span<uint8_t> rom_view)
{
    RomHeader header;
    std::memcpy(&header, rom_view.data() + RomHeader::ENTRY_POINT, sizeof(RomHeader));

    auto& logger = logger::Logger::instance();
    logger.log("Title Game: \t{}", std::string(header.title.begin(), header.title.end()));
    logger.log("ROM Size: \t{}", static_cast<size_t>(header.rom_size));
    logger.log("RAM Size: \t{}", static_cast<size_t>(header.ram_size));
    logger.log("OLC: \t{:#x}", static_cast<size_t>(header.old_license_code));
    logger.log("G. CSUM: \t{:#x}", static_cast<size_t>(header.global_checksum));

    const auto test = compute_checksum(rom_view) & 0xFF ? "PASSED" : "FAILED";
    logger.log(std::format("Checksum: {:#x}, => ({})\n", static_cast<int>(header.checksum), test));

    return header;
}

class Cartridge
{
   public:
    Cartridge() = delete;

    explicit Cartridge(std::string filename) : filename_(std::move(filename))
    {
        rom_data_ = gameboy::utils::read_rom(filename_);
        header_ = gameboy::cartridge::from_rom(rom_data_);
        setup_banks();
        battery = has_battery();
        if (has_battery()) {
            battery_load();
        }
    }

    auto header() const noexcept -> const RomHeader&
    {
        return header_;
    }

    auto read(uint16_t address) -> uint8_t
    {
        if (!is_mbc1() || address < ROM_BASE_ADDRESS) {
            return rom_data_[address];
        }

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

        return rom_bank_x[address - ROM_BASE_ADDRESS];
    }

    void write(uint16_t address, uint8_t value)
    {
        // ONLY MBC1
        if (!is_mbc1()) {
            return;
        }

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
            rom_bank_x = rom_data_.data() + (ROM_BASE_ADDRESS * rom_bank_value);
        }

        // RAM BANK
        if (masked_address == 0x4000) {
            ram_bank_value = value & 0b11;

            if (ram_banking) {
                if (need_save) {
                    battery_save();
                }
                ram_bank = ram_banks[ram_bank_value].data();
            }
        }

        // Range 0x6000
        if (masked_address == 0x6000) {
            banking_mode = value & 1;  // last bit
            ram_banking = value & 1;

            if (ram_banking) {
                if (need_save) {
                    battery_save();
                }
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

            if (battery) {
                need_save = true;
            }
        }
    }

   private:
    void battery_load()
    {
    }

    void battery_save()
    {
    }

    void setup_banks()
    {
        for (uint8_t i = 0; i < NUMBER_RAM_BANKS; ++i) {
            // 2 means 1 bank
            auto ram_size = header_.ram_size;

            bool check_for_ram = (ram_size == 2 && i == 0) || (ram_size == 3 && i < 4) || (ram_size == 4 && i < 16) ||
                                 (ram_size == 5 && i < 8);
            if (check_for_ram) {
                std::cout << "Initializing RAM bank " << static_cast<int>(i) << std::endl;
                ram_banks[i].resize(RAM_BANK_SIZE, 0);
            }
        }

        ram_bank = ram_banks[0].data();
        rom_bank_x = rom_data_.data() + ROM_BASE_ADDRESS;  // rom bank 1
    }

    bool is_mbc1() const noexcept
    {
        auto num_bank = std::to_underlying(header_.cartridge_type);
        return num_bank >= 1 && num_bank <= 3;
    }

    bool has_battery() const noexcept
    {
        auto num_bank = std::to_underlying(header_.cartridge_type);
        return num_bank == 3;  // only for MBC1
    }

    std::vector<uint8_t> rom_data_;
    RomHeader header_;
    std::string filename_;

    // MBC1 related data
    bool ram_enabled;
    bool ram_banking;

    uint8_t* rom_bank_x{nullptr};
    uint8_t banking_mode;

    uint8_t rom_bank_value;
    uint8_t ram_bank_value;

    uint8_t* ram_bank{nullptr};

    static constexpr uint8_t NUMBER_RAM_BANKS{16};
    RamBank ram_banks[NUMBER_RAM_BANKS];  // number of banks

    // battery
    bool battery;
    bool need_save{false};  // should save data

    static constexpr uint16_t RAM_BANK_SIZE{0x2000};
    static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};
};
}  // namespace gameboy::cartridge
