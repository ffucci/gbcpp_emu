#pragma once

#include <sys/types.h>
#include <cstdint>
#include <format>
#include <iterator>
#include <stdexcept>
#include <vector>
#include <fstream>
#include "logger.h"

namespace gameboy::utils {
template <class Elem, class Traits>
inline void hex_dump(
    const void *aData, std::size_t aLength, std::basic_ostream<Elem, Traits> &aStream, std::size_t aWidth)
{
    const char *const start = static_cast<const char *>(aData);
    const char *const end = start + aLength;
    const char *line = start;
    while (line != end) {
        aStream.width(4);
        aStream.fill('0');
        aStream << logger::LogColors::ERROR << std::hex << line - start << " : ";
        std::size_t lineLength = std::min(aWidth, static_cast<std::size_t>(end - line));
        for (std::size_t pass = 1; pass <= 2; ++pass) {
            for (const char *next = line; next != end && next != line + aWidth; ++next) {
                char ch = *next;
                switch (pass) {
                    case 1:
                        aStream << (ch < 32 ? '.' : ch);
                        break;
                    case 2:
                        if (next != line) aStream << " ";
                        aStream.width(2);
                        aStream.fill('0');
                        aStream << std::hex << std::uppercase << static_cast<int>(static_cast<unsigned char>(ch));
                        break;
                }
            }
            if (pass == 1 && lineLength != aWidth) aStream << std::string(aWidth - lineLength, ' ');
            aStream << " ";
        }
        aStream << std::endl;
        line = line + lineLength;
    }
}

inline auto read_rom(const std::string &file_name) -> std::vector<uint8_t>
{
    std::ifstream in_file(file_name.c_str(), std::ios::binary | std::ios::ate);

    if (!in_file.good()) {
        throw std::runtime_error("Cannot open the rom file.");
    }

    auto eof = in_file.tellg();
    auto file_size = static_cast<size_t>(eof);
    auto &logger = logger::Logger::instance();
    logger.log(std::format("Game file size is {}", file_size));
    std::vector<uint8_t> result(file_size);

    in_file.seekg(0, std::ios::beg);  // Go back to the beginning of the file.
    in_file.read(std::bit_cast<char *>(result.data()), eof);

    hex_dump(result.data(), 128, std::cout, 16);
    std::cout << std::endl;
    return result;
}
}  // namespace gameboy::utils
