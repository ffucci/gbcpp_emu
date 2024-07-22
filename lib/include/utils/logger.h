#pragma once

#include <chrono>
#include <iostream>
#include <stop_token>
#include <thread>
#include <format>

#include <boost/lockfree/spsc_queue.hpp>

namespace gameboy::logger {
struct LogColors
{
    static constexpr std::string_view TRACE = "\033[1;30m";
    static constexpr std::string_view DEBUG = "\033[1;37m";
    static constexpr std::string_view UNIMPLEMENTED = "\033[1;35m";
    static constexpr std::string_view INFO = "\033[1;34m";
    static constexpr std::string_view WARNING = "\033[1;33m";
    static constexpr std::string_view ERROR = "\033[1;31m";
    static constexpr std::string_view RESET = "\033[0m";
};

class Logger
{
   public:
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    static Logger& instance()
    {
        static Logger inst_;
        return inst_;
    }

    void log(std::string current)
    {
        buffer_.push(std::move(current));
    }

    void log(std::string_view fmt_in, auto... vars)
    {
        // buffer_.push(std::vformat(fmt_in, std::make_format_args(vars...)));
        std::cout << std::vformat(fmt_in, std::make_format_args(vars...)) << std::endl;
    }

   private:
    Logger()
    {
        logger_thread_ = std::jthread([this](std::stop_token token) {
            while (!token.stop_requested()) {
                std::string res;
                while (buffer_.pop(res)) {
                    std::cout << LogColors::INFO << res << LogColors::RESET << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::microseconds(10));
            }
        });
    }
    ~Logger() = default;

    boost::lockfree::spsc_queue<std::string> buffer_{1024};
    std::jthread logger_thread_;
};
}  // namespace gameboy::logger
