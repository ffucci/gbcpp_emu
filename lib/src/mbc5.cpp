#include "cartridge/mbc5.h"

#include <algorithm>
#include <utility>

namespace gameboy::cartridge {
namespace {

constexpr uint16_t ROM_BANK_0_START{0x0000};
constexpr uint16_t ROM_BANK_0_END{0x4000};
constexpr uint16_t SWITCHABLE_ROM_BANK_START{0x4000};
constexpr uint16_t SWITCHABLE_ROM_BANK_END{0x8000};
constexpr uint16_t RAM_BANK_START{0xA000};
constexpr uint16_t RAM_BANK_END{0xC000};

constexpr uint16_t RAM_ENABLE_END{0x2000};
constexpr uint16_t ROM_BANK_LOW_START{0x2000};
constexpr uint16_t ROM_BANK_LOW_END{0x3000};
constexpr uint16_t ROM_BANK_HIGH_START{0x3000};
constexpr uint16_t ROM_BANK_HIGH_END{0x4000};
constexpr uint16_t RAM_BANK_SELECT_START{0x4000};
constexpr uint16_t RAM_BANK_SELECT_END{0x6000};

constexpr uint8_t RAM_ENABLE_VALUE{0x0A};
constexpr uint8_t LOW_NIBBLE_MASK{0x0F};
constexpr uint8_t ROM_BANK_HIGH_BIT_MASK{0x01};
constexpr uint16_t ROM_BANK_LOW_BITS_MASK{0x0FF};
constexpr uint16_t ROM_BANK_HIGH_BIT{0x100};

}  // namespace

MBC5::MBC5(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
    : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
{
}

uint8_t MBC5::read(uint16_t address) const
{
    if (address_in_range(address, ROM_BANK_0_START, ROM_BANK_0_END)) {
        return rom_data_[address];
    }

    if (address_in_range(address, SWITCHABLE_ROM_BANK_START, SWITCHABLE_ROM_BANK_END)) {
        const uint16_t addr_within_bank = address - SWITCHABLE_ROM_BANK_START;
        const uint32_t bank_offset = ROM_BASE_ADDRESS * selected_rom_bank();
        return rom_data_[bank_offset + addr_within_bank];
    }

    if (address_in_range(address, RAM_BANK_START, RAM_BANK_END)) {
        if (!ram_enabled_ || ram_data_.empty()) {
            return 0xFF;
        }

        const uint16_t addr_within_bank = address - RAM_BANK_START;
        const uint32_t bank_offset = RAM_BANK_SIZE * selected_ram_bank();
        return ram_data_[bank_offset + addr_within_bank];
    }

    return 0xFF;
}

void MBC5::write(uint16_t address, uint8_t value)
{
    if (address < RAM_ENABLE_END) {
        ram_enabled_ = ((value & LOW_NIBBLE_MASK) == RAM_ENABLE_VALUE);
        return;
    }

    if (address_in_range(address, ROM_BANK_LOW_START, ROM_BANK_LOW_END)) {
        rom_idx_ = (rom_idx_ & ROM_BANK_HIGH_BIT) | value;
        return;
    }

    if (address_in_range(address, ROM_BANK_HIGH_START, ROM_BANK_HIGH_END)) {
        rom_idx_ = (rom_idx_ & ROM_BANK_LOW_BITS_MASK) | ((value & ROM_BANK_HIGH_BIT_MASK) << 8);
        return;
    }

    if (address_in_range(address, RAM_BANK_SELECT_START, RAM_BANK_SELECT_END)) {
        ram_idx_ = value & LOW_NIBBLE_MASK;
        return;
    }

    if (address_in_range(address, RAM_BANK_START, RAM_BANK_END)) {
        if (!ram_enabled_ || ram_data_.empty()) {
            return;
        }

        const uint16_t addr_within_bank = address - RAM_BANK_START;
        const uint32_t bank_offset = RAM_BANK_SIZE * selected_ram_bank();
        ram_data_[bank_offset + addr_within_bank] = value;

        if (battery_saver) {
            battery_saver->save(filename_, ram_data_.data() + bank_offset);
        }
    }
}

auto MBC5::selected_rom_bank() const noexcept -> uint16_t
{
    const auto num_banks = static_cast<uint16_t>(std::max<size_t>(1, rom_data_.size() / ROM_BASE_ADDRESS));
    return rom_idx_ % num_banks;
}

auto MBC5::selected_ram_bank() const noexcept -> uint16_t
{
    const auto num_banks = static_cast<uint16_t>(std::max<size_t>(1, ram_data_.size() / RAM_BANK_SIZE));
    return ram_idx_ % num_banks;
}

}  // namespace gameboy::cartridge
