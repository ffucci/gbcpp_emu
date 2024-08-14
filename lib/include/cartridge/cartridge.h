#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "cartridge/cartridge_metadata.h"
#include "cartridge/mbc1_cart.h"
#include "utils/logger.h"

namespace gameboy::cartridge {

class Cartridge
{
   public:
    explicit Cartridge(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : filename_(std::move(filename)), rom_data_(std::move(rom_data)), header_(std::move(metadata))
    {
        // setup_banks();
        // battery = has_battery();
        // if (has_battery()) {
        //     battery_load();
        // }
    }

    virtual auto read(uint16_t address) const -> uint8_t = 0;
    virtual void write(uint16_t address, uint8_t value) = 0;

    Cartridge() = delete;

    virtual ~Cartridge() = default;

   protected:
    std::string filename_;
    RomBank rom_data_;
    RamBank ram_data_;

    CartridgeMetadata header_;

    static constexpr uint16_t RAM_BANK_SIZE{0x2000};
    static constexpr uint16_t ROM_BASE_ADDRESS{0x4000};
};

class NoMBC : public Cartridge
{
   public:
    explicit NoMBC(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
        : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
    {
    }

    uint8_t read(uint16_t address) const override
    {
        return rom_data_[address];
    }

    void write(uint16_t address, uint8_t) override
    {
        auto& logger = logger::Logger::instance();
        logger.log("Cannot write in NoMBC mode...");
    }
};

// class MBC1 : public Cartridge
// {
//    public:
//     explicit MBC1(std::string filename, RomBank rom_data, CartridgeMetadata metadata)
//         : Cartridge(std::move(filename), std::move(rom_data), std::move(metadata))
//     {
//     }

//     uint8_t read(uint16_t address) const override
//     {
//         return rom_data_[address];
//     }

//     void write(uint16_t address, uint8_t) override
//     {
//         auto& logger = logger::Logger::instance();
//         logger.log("Cannot write in NoMBC mode...");
//     }

//     // battery
//     bool battery;
//     bool need_save{false};  // should save data
// };

inline std::unique_ptr<Cartridge> get_cartridge(std::string filename)
{
    auto rom_data = gameboy::utils::read_rom(filename);
    auto header = gameboy::cartridge::from_rom(rom_data);
    return std::make_unique<NoMBC>(std::move(filename), rom_data, header);
}

}  // namespace gameboy::cartridge