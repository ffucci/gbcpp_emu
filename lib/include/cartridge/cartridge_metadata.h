#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <span>
#include <type_traits>
#include <utility>

#include "utils/logger.h"
#include "utils/utils.h"

namespace gameboy::cartridge {

static constexpr uint16_t RAM_BANK_SIZE{0x2000};
static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};

// Add the cartridge type
enum class CartridgeType : uint8_t
{
    RomOnly = 0,
    MBC1,
    MBC1_RAM,
    MBC1_RAM_BATTERY,
    MBC2 = 5,
    MBC2_BATTERY,
    // TODO: Add all of them (MBC1 for now is enough)
    ROM_RAM = 8,
    ROM_RAM_BATTERY,

    MBC3_TIMER_BATTERY = 0xF,
    MBC3_TIMER_RAM_BATTERY,
    MBC3,
    MBC3_RAM,
    MBC3_RAM_BATTERY,
    MBC4,
    MBC5,
    Unknown
};

template <CartridgeType... ALL>
static constexpr bool find_cart_type(CartridgeType h)
{
    std::array all_types = {ALL...};
    return std::find(std::begin(all_types), std::end(all_types), h) != std::end(all_types);
}

using RomBank = std::vector<uint8_t>;
using RamBank = std::vector<uint8_t>;

struct CartridgeMetadata
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

    bool is_mbc1() const noexcept
    {
        auto num_bank = std::to_underlying(cartridge_type);
        return num_bank >= 1 && num_bank <= 3;
    }

    bool is_mbc3() const noexcept
    {
        auto num_bank = std::to_underlying(cartridge_type);
        return num_bank >= 15 && num_bank <= 19;
    }

    bool has_battery() const noexcept
    {
        return find_cart_type<
            CartridgeType::MBC1_RAM_BATTERY, CartridgeType::MBC2_BATTERY, CartridgeType::MBC3_RAM_BATTERY,
            CartridgeType::MBC3_TIMER_BATTERY>(cartridge_type);
        // return cartridge_type == CartridgeType::MBC1_RAM_BATTERY || cartridge_type == CartridgeType::MBC2_BATTERY ||
        //        cartridge_type == CartridgeType::MBC3_RAM_BATTERY || cartridge_type ==
        //        CartridgeType::MBC3_TIMER_BATTERY;
    }
};

inline auto compute_checksum(std::span<uint8_t> rom_view) -> uint8_t
{
    uint8_t checksum = 0;
    for (uint16_t address = 0x0134; address <= 0x014C; address++) {
        checksum = checksum - rom_view[address] - 1;
    }

    return checksum;
}
inline CartridgeMetadata from_rom(std::span<uint8_t> rom_view)
{
    CartridgeMetadata header;
    std::memcpy(&header, rom_view.data() + CartridgeMetadata::ENTRY_POINT, sizeof(CartridgeMetadata));

    auto& logger = logger::Logger::instance();
    logger.log("Title Game: \t{}", std::string(header.title.begin(), header.title.end()));
    logger.log("ROM Size: \t{}", static_cast<size_t>(header.rom_size));
    logger.log("RAM Size: \t{}", static_cast<size_t>(header.ram_size));
    logger.log("OLC: \t{:#x}", static_cast<size_t>(header.old_license_code));
    logger.log("G. CSUM: \t{:#x}", static_cast<size_t>(header.global_checksum));
    logger.log("Cartridge type: \t{:x}", std::to_underlying(header.cartridge_type));
    const auto test = compute_checksum(rom_view) & 0xFF ? "PASSED" : "FAILED";
    logger.log(std::format("Checksum: {:#x}, => ({})\n", static_cast<int>(header.checksum), test));

    return header;
}

// class Cartridge
// {
//    public:
//     Cartridge() = delete;

//     explicit Cartridge(std::string filename) : filename_(std::move(filename))
//     {
//         rom_data_ = gameboy::utils::read_rom(filename_);
//         header_ = gameboy::cartridge::from_rom(rom_data_);
//         // setup_banks();
//         // battery = has_battery();
//         // if (has_battery()) {
//         //     battery_load();
//         // }
//         extension_ = std::make_unique<MBC1>(filename_, rom_data_, header_.ram_size);
//     }

//     auto header() const noexcept -> const CartridgeMetadata&
//     {
//         return header_;
//     }

//     auto read(uint16_t address) -> uint8_t
//     {
//         if (!is_mbc1() || address < ROM_BASE_ADDRESS) {
//             return rom_data_[address];
//         }

//         return extension_->read(address);
//     }

//     void write(uint16_t address, uint8_t value)
//     {
//         // ONLY MBC1
//         if (!is_mbc1()) {
//             return;
//         }

//         extension_->write(address, value);
//     }

//    private:
//     bool is_mbc1() const noexcept
//     {
//         auto num_bank = std::to_underlying(header_.cartridge_type);
//         return num_bank >= 1 && num_bank <= 3;
//     }

//     bool has_battery() const noexcept
//     {
//         auto num_bank = std::to_underlying(header_.cartridge_type);
//         return num_bank == 3;  // only for MBC1
//     }

//     std::vector<uint8_t> rom_data_;
//     CartridgeMetadata header_;
//     std::string filename_;

//     std::unique_ptr<MBC1> extension_;

//     static constexpr uint16_t RAM_BANK_SIZE{0x2000};
//     static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};
// };
}  // namespace gameboy::cartridge
