#pragma once

#include "cartridge/cartridge.h"
#include "cartridge/mbc1.h"
#include "cartridge/mbc3.h"
#include "cartridge/no_mbc.h"

namespace gameboy::cartridge {

inline std::unique_ptr<Cartridge> get_cartridge(std::string filename)
{
    auto rom_data = gameboy::utils::read_rom(filename);
    auto header = gameboy::cartridge::from_rom(rom_data);

    if (header.is_mbc1()) {
        return std::make_unique<MBC1>(std::move(filename), rom_data, header);
    }

    if (header.is_mbc3()) {
        return std::make_unique<MBC3>(std::move(filename), rom_data, header);
    }

    return std::make_unique<NoMBC>(std::move(filename), rom_data, header);
}

}  // namespace gameboy::cartridge