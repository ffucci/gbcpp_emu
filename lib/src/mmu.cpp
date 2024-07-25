#include "mmu/mmu.h"

namespace gameboy::memory {

auto MMU::read(uint16_t address) const -> uint8_t
{
    auto& logger = logger::Logger::instance();

    if (address < HRAM_LIMIT) {
        return cartridge_.read(address);
    }

    if (address < CHR_RAM_LIMIT) {
        logger.log("Unsupported BUS read @{:4X}", address);
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        exit(-7);
    }

    if (address < 0xC000) {
        return cartridge_.read(address);
    }

    if (address < 0xE000) {
        if constexpr (DEBUG) {
            logger.log("bus_read({:04X})", address);
        }
        // We go into the WRAM
        return ram_.wram_read(address);
    }

    if (address < 0xFE00) {
        // Reserved echo RAM
        return 0;
    }

    if (address < 0xFEA0) {
        auto& logger = logger::Logger::instance();
        logger.log("UNSUPPORTED bus_read(@{:4X})", address);
        return 0;
    }

    if (address < 0xFF00) {
        // unused section
        return 0;
    }

    if (address < 0xFF80) {
        return device_.read(address);
    }

    if (address == 0xFFFF) {
        return int_enable_register_;
    }

    return ram_.hram_read(address);
}

auto MMU::read16(uint16_t address) const -> uint16_t
{
    auto low = read(address);
    auto hi = read(address + 1);
    return low | (hi << 8);
}
auto MMU::write(uint16_t address, uint8_t value) -> void
{
    auto& logger = logger::Logger::instance();

    if (address < HRAM_LIMIT) {
        cartridge_.write(address, value);
        return;
    }

    if (address < CHR_RAM_LIMIT) {
        // throw std::runtime_error(std::format("NOT IMPL {:X}", address));
        logger.log("UNSUPPORTED bus_write({:04X})", address);
        return;
    }

    if (address < 0xC000) {
        cartridge_.write(address, value);
        return;
    }

    if (address < 0xE000) {
        // WRAM write
        if constexpr (DEBUG) {
            logger.log("bus_write({:04X}, {:02X})", address, value);
        }
        ram_.wram_write(address, value);
        return;
    }

    if (address < 0xFE00) {
        // Reserved echo RAM
        return;
    }

    if (address < 0xFEA0) {
        auto& logger = logger::Logger::instance();
        logger.log("Unsuppored bus write {:04X}", address);
        return;
    }

    if (address < 0xFF00) {
        // unused section
        auto& logger = logger::Logger::instance();
        logger.log("Unsuppored bus write {:04X}", address);
        return;
    }

    if (address < 0xFF80) {
        device_.write(address, value);
        return;
    }

    if (address == 0xFFFF) {
        logger.log("SET ENABLE {:04X} - {:04X}", address, value);
        int_enable_register_ = value;
        return;
    }

    ram_.hram_write(address, value);
}
auto MMU::write16(uint16_t address, uint16_t value) -> void
{
    write(address + 1, (value >> 8) & 0xFF);
    write(address, value & 0xFF);
}
auto MMU::ie_register() -> uint8_t
{
    return int_enable_register_;
}
}  // namespace gameboy::memory