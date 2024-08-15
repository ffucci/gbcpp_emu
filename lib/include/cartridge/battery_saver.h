#pragma once

#include <iostream>
#include <string_view>
#include <format>
#include <fstream>
#include "cartridge/cartridge_metadata.h"

namespace gameboy::cartridge {

class BatterySaver
{
   public:
    void load(std::string_view filename, uint8_t* ram_bank)
    {
        if (!ram_bank) {
            return;
        }
        auto fname = std::format("{}.battery", filename);
        auto in_file = std::ifstream(fname, std::ios::in | std::ios::binary);

        in_file.read(std::bit_cast<char*>(ram_bank), 0x2000);
        in_file.close();
    }

    void save(std::string_view filename, const uint8_t* ram_bank)
    {
        if (!ram_bank) {
            return;
        }
        std::cout << "Save game... " << std::endl;
        auto fname = std::format("{}.battery", filename);
        auto file = std::ofstream(fname, std::ios::out | std::ios::binary);
        file.write(std::bit_cast<char*>(ram_bank), 0x2000);
        file.close();
    }
};
}  // namespace gameboy::cartridge