#pragma once

#include <cstdint>
#include <iostream>
#include <string_view>
#include <format>
#include <fstream>
#include "cartridge/cartridge.h"
#include "cartridge/cartridge_metadata.h"

namespace gameboy::cartridge {

class Battery
{
   public:
    void load(std::string_view filename, std::span<uint8_t> ram_bank)
    {
        if (ram_bank.empty()) {
            return;
        }

        std::cout << "Loading saved data... " << std::endl;
        auto fname = std::format("{}.battery", filename);
        auto in_file = std::ifstream(fname, std::ios::in | std::ios::binary);

        in_file.read(std::bit_cast<char*>(ram_bank.data()), RAM_BANK_SIZE);
        in_file.close();
    }

    void save(std::string_view filename, const uint8_t* ram_bank)
    {
        if (!ram_bank) {
            std::cout << "Cannot save " << std::endl;
            return;
        }

        auto time = std::chrono::system_clock::now();
        auto current_timestamp = time.time_since_epoch().count();
        if (last_save_timestamp_ == 0) {
            last_save_timestamp_ = current_timestamp;
            return;
        }

        if (((current_timestamp - last_save_timestamp_) > (10ul * std::nano::den))) {
            std::cout << "Passed " << std::dec << (current_timestamp - last_save_timestamp_) / std::nano::den << "sec "
                      << std::endl;
            last_save_timestamp_ = current_timestamp;
            std::cout << "Save game... " << std::endl;
            auto fname = std::format("{}.battery", filename);
            auto file = std::ofstream(fname, std::ios::out | std::ios::binary);
            file.write(std::bit_cast<char*>(ram_bank), RAM_BANK_SIZE);
            file.close();
        }
    }

   private:
    // We can sort of rate limit the when to save
    uint64_t last_save_timestamp_{0};
};
}  // namespace gameboy::cartridge