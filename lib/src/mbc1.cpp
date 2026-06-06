#include "cartridge/mbc1.h"

#include <utility>

namespace gameboy::cartridge {
namespace {

constexpr uint16_t ADDRESS_RANGE_MASK{0xE000};
constexpr uint16_t ROM_BANK_0_END{0x4000};
constexpr uint16_t RAM_ENABLE_END{0x2000};
constexpr uint16_t ROM_BANK_SELECT_RANGE{0x2000};
constexpr uint16_t RAM_BANK_SELECT_RANGE{0x4000};
constexpr uint16_t BANKING_MODE_SELECT_RANGE{0x6000};
constexpr uint16_t EXTERNAL_RAM_RANGE{0xA000};
constexpr uint16_t EXTERNAL_RAM_START{0xA000};

constexpr uint8_t RAM_ENABLE_VALUE{0x0A};
constexpr uint8_t LOW_NIBBLE_MASK{0x0F};
constexpr uint8_t ROM_BANK_MASK{0x1F};
constexpr uint8_t RAM_BANK_MASK{0x03};
constexpr uint8_t BANKING_MODE_MASK{0x01};
constexpr uint16_t FIRST_SWITCHABLE_ROM_BANK{1};

}  // namespace

MBC1::MBC1(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
    : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
{
    rom_idx_ = FIRST_SWITCHABLE_ROM_BANK;
    std::cout << "Size of ram: " << ram_data_.size() << std::endl;
}

uint8_t MBC1::read(uint16_t address) const
{
    if (address < ROM_BANK_0_END) {
        return rom_data_[address];
    }

    const uint16_t masked_address = address & ADDRESS_RANGE_MASK;

    if (masked_address == EXTERNAL_RAM_RANGE) {
        if (!ram_enabled_) {
            return 0xFF;
        }

        const uint16_t addr_within_bank = address - EXTERNAL_RAM_START;
        const uint16_t bank_offset = RAM_BANK_SIZE * ram_idx_;
        return ram_data_[bank_offset + addr_within_bank];
    }

    const uint16_t addr_within_bank = address - ROM_BASE_ADDRESS;
    const uint32_t bank_offset = ROM_BASE_ADDRESS * rom_idx_;
    return rom_data_[bank_offset + addr_within_bank];
}

void MBC1::write(uint16_t address, uint8_t value)
{
    const uint16_t masked_address = address & ADDRESS_RANGE_MASK;

    if (address < RAM_ENABLE_END) {
        ram_enabled_ = ((value & LOW_NIBBLE_MASK) == RAM_ENABLE_VALUE);
    }

    if (masked_address == ROM_BANK_SELECT_RANGE) {
        value = (value == 0) ? FIRST_SWITCHABLE_ROM_BANK : value;
        rom_idx_ = value & ROM_BANK_MASK;
        return;
    }

    if (masked_address == RAM_BANK_SELECT_RANGE) {
        ram_idx_ = value & RAM_BANK_MASK;

        if (need_save_) {
            save_to_battery();
        }
        return;
    }

    if (masked_address == BANKING_MODE_SELECT_RANGE) {
        ram_banking = value & BANKING_MODE_MASK;

        if (ram_banking) {
            save_to_battery();
        }
    }

    if (masked_address == EXTERNAL_RAM_RANGE) {
        if (!ram_enabled_) {
            return;
        }

        const uint16_t addr_within_bank = address - EXTERNAL_RAM_START;
        const uint32_t bank_offset = RAM_BANK_SIZE * ram_idx_;
        ram_data_[bank_offset + addr_within_bank] = value;

        if (header_.has_battery()) {
            need_save_ = true;
        }
    }

    save_to_battery();
}

MBC1::~MBC1()
{
    save_to_battery();
}

void MBC1::save_to_battery()
{
    if (battery_saver) {
        const uint32_t bank_offset = RAM_BANK_SIZE * ram_idx_;
        auto ram_bank = ram_data_.data() + bank_offset;
        battery_saver->save(filename_, ram_bank);
    }
}

}  // namespace gameboy::cartridge
