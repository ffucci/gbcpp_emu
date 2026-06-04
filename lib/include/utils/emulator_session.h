#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "cartridge/cartridge.h"
#include "cpu/cpu.h"

namespace gameboy::app {

class EmulatorSession
{
   public:
    EmulatorSession() = default;
    EmulatorSession(const EmulatorSession&) = delete;
    EmulatorSession& operator=(const EmulatorSession&) = delete;

    ~EmulatorSession()
    {
        stop();
    }

    bool load_rom(const std::string& path);
    void stop();

    auto cpu() noexcept -> cpu::CPU*
    {
        std::scoped_lock lock(mutex_);
        return cpu_.get();
    }

    [[nodiscard]] auto running() const noexcept -> bool
    {
        std::scoped_lock lock(mutex_);
        return cpu_ != nullptr;
    }

    [[nodiscard]] auto current_rom_path() const -> std::string
    {
        std::scoped_lock lock(mutex_);
        return current_rom_path_;
    }

   private:
    mutable std::mutex mutex_;
    std::unique_ptr<cartridge::Cartridge> cartridge_;
    std::unique_ptr<cpu::CPU> cpu_;
    std::jthread cpu_thread_;
    std::string current_rom_path_;
};

}  // namespace gameboy::app
