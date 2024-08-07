#pragma once

#include <iostream>
#include <string_view>
#include <format>
#include <fstream>

namespace gameboy::cartridge {

struct BatterySaver
{
    template <typename T>
    static void battery_load(std::string_view filename, T& loader)
    {
        auto fname = std::format("{}.battery", filename);
        auto in_file = std::ifstream(fname, std::ios::in | std::ios::binary);
        in_file.read(std::bit_cast<char*>(loader.ram_bank), T::RAM_BANK_SIZE);
        in_file.close();
    }

    template <typename T>
    void battery_save(std::string_view filename, const T& loader)
    {
        if (!loader.ram_bank) {
            return;
        }
        std::cout << "Save game... " << std::endl;
        auto fname = std::format("{}.battery", filename_);
        auto file = std::ofstream(fname, std::ios::out | std::ios::binary);
        file.write(std::bit_cast<char*>(ram_bank), RAM_BANK_SIZE);
        file.close();
    }
};
}  // namespace gameboy::cartridge