#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <span>
#include <memory>

#include "utils/logger.h"

enum class CartridgeType : uint8_t
{

};

struct RomHeader
{
    static constexpr uint8_t NUM_ENTRY_BITS{4};
    static constexpr uint8_t NUM_LOGO_BITS{0x30};

    uint8_t entry[NUM_ENTRY_BITS];
    uint8_t logo[NUM_LOGO_BITS];
    char title[16];
    uint16_t new_licensee_code;
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

inline RomHeader from_rom(std::span<uint8_t> rom_view)
{
    RomHeader header;
    std::memcpy(&header, rom_view.data() + 0x100, sizeof(RomHeader));

    auto& logger = Logger::instance();
    logger.log("Title Game: \t{}", header.title);
    logger.log("ROM Size: \t{}", static_cast<size_t>(header.rom_size));
    logger.log("RAM Size: \t{}", static_cast<size_t>(header.ram_size));
    logger.log("G. CSUM: \t{:#x}", static_cast<size_t>(header.global_checksum));

    // std::cout << "RAM Size: " << std::dec << static_cast<size_t>(header.ram_size) << std::endl;
    // std::cout << "Global Checksum: " << std::hex << header.global_checksum << std::endl;
    uint8_t checksum = 0;
    for (uint16_t address = 0x0134; address <= 0x014C; address++) {
        checksum = checksum - rom_view[address] - 1;
    }

    auto test = checksum & 0xFF ? "PASSED" : "FAILED";
    logger.log(std::format("Checksum: ({}), {}", static_cast<int>(header.checksum), test));
    return header;
}