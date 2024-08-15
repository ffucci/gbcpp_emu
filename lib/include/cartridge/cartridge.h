#pragma once

#include <sys/types.h>
#include <cstdint>
#include <ios>
#include <memory>
#include <stdexcept>
#include <string>

#include "cartridge/battery.h"
#include "cartridge/cartridge_metadata.h"
#include "utils/logger.h"

namespace gameboy::cartridge {

inline bool address_in_range(uint16_t address, uint16_t left, uint16_t right)
{
    return address >= left && address < right;
}

class Cartridge
{
   public:
    explicit Cartridge(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : filename_(std::move(filename)), rom_data_(std::move(rom_data)), header_(std::move(metadata))
    {
        setup_banks(header_.ram_size);
        has_battery_ = header_.has_battery();
        if (has_battery_) {
            std::cout << "Game has battery " << std::endl;
            battery_saver = std::make_unique<Battery>();
            battery_saver->load(filename_, ram_data_.data());
        }
    }

    virtual auto read(uint16_t address) const -> uint8_t = 0;
    virtual void write(uint16_t address, uint8_t value) = 0;

    Cartridge() = delete;

    virtual ~Cartridge() = default;

   protected:
    void setup_banks(uint8_t ram_size)
    {
        std::cout << "Setup banks: " << static_cast<int>(ram_size) << std::endl;
        switch (ram_size) {
            case 2: {
                ram_data_.resize(RAM_BANK_SIZE);
                break;
            };

            case 3: {
                ram_data_.resize(4 * RAM_BANK_SIZE);
                break;
            }

            case 4: {
                ram_data_.resize(16 * RAM_BANK_SIZE);
                break;
            }

            case 5: {
                ram_data_.resize(8 * RAM_BANK_SIZE);
                break;
            }
        }
    }

    std::string filename_;
    RomBank rom_data_;
    RamBank ram_data_{};

    CartridgeMetadata header_;

    bool ram_enabled_{false};
    // battery
    bool has_battery_{false};
    bool need_save_{false};  // should save data
    std::unique_ptr<Battery> battery_saver;

    uint16_t rom_idx_{0};
    uint16_t ram_idx_{0};

    static constexpr uint16_t RAM_BANK_SIZE{0x2000};
    static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};
};

}  // namespace gameboy::cartridge