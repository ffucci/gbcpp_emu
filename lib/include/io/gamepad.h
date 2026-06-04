#pragma once

#include <cstdint>
namespace gameboy::io {

class GamePad
{
   public:
    enum class Button : uint16_t
    {
        B = 0,
        A = 1,
        Select = 2,
        Start = 3,
        Right = 4,
        Left = 5,
        Up = 6,
        Down = 7,
    };

    [[nodiscard]] auto button_selected() const noexcept -> bool
    {
        return context_ & BUTTON_SELECT_MASK;
    }

    [[nodiscard]] auto direction_selected() const noexcept -> bool
    {
        return context_ & DIRECTION_SELECT_MASK;
    }

    [[nodiscard]] auto pressed(Button button) const noexcept -> bool
    {
        return context_ & button_mask(button);
    }

    void set_pressed(Button button, bool pressed) noexcept;

    [[nodiscard]] auto encoded_context() const noexcept -> uint16_t
    {
        return context_;
    }

    void set_encoded_context(uint16_t context) noexcept
    {
        context_ = context;
    }

    void set_selection(uint8_t value);

    [[nodiscard]] auto get_output() const noexcept -> uint8_t;

   private:
    [[nodiscard]] static constexpr auto button_mask(Button button) noexcept -> uint16_t
    {
        return 1 << static_cast<uint16_t>(button);
    }

    void set_bit(uint16_t mask, bool enabled) noexcept;

    uint16_t context_{0};

    static constexpr uint16_t BUTTON_SELECT_MASK{1 << 8};
    static constexpr uint16_t DIRECTION_SELECT_MASK{1 << 9};
};

}  // namespace gameboy::io
