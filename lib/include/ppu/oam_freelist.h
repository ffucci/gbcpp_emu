#pragma once

#include <sys/types.h>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>

#include "ppu/oam_data.h"

namespace gameboy::ppu {

template <typename Entry>
struct Node
{
    Entry entry;
    Node<Entry>* next{nullptr};
};

template <size_t SIZE, size_t FETCHED_ENTRY = 3>
struct OAMFreeList
{
    bool insert(const OAMEntry& e) noexcept
    {
        auto* node = &nodes_[number_of_sprite_lines++];
        node->entry = e;
        node->next = nullptr;
        if (freelist_head_ == nullptr || freelist_head_->entry.x > node->entry.x) {
            node->next = freelist_head_;
            freelist_head_ = node;
            return true;
        }

        auto* le = freelist_head_;
        auto* prev = le;
        while (le) {
            if (le->entry.x > node->entry.x) {
                prev->next = node;
                node->next = le;
                break;
            }

            if (!le->next) {
                le->next = node;
                break;
            }

            prev = le;
            le = le->next;
        }

        return false;
    }

    auto sprite_lines() const noexcept -> uint8_t
    {
        return number_of_sprite_lines;
    }

    bool has_freelist_head() const noexcept
    {
        return freelist_head_ != nullptr;
    }

    void load_sprite_tile(uint8_t fetch_x, uint8_t scroll_x)
    {
        auto* le = freelist_head_;

        const auto has_to_fetch = [fetch_x = fetch_x](int sprite_x) {
            return sprite_x >= fetch_x && sprite_x < (fetch_x + 8);
        };

        while (le) {
            int sprite_x = (le->entry.x - 8) + (scroll_x % 8);
            if (has_to_fetch(sprite_x) || has_to_fetch(sprite_x + 8)) {
                fetched_entries[fetch_entry_count++] = le->entry;
            }

            le = le->next;

            if (!le || fetch_entry_count >= 3) {
                break;
            }
        }
    }

    auto fetched_entry_count() noexcept -> uint8_t&
    {
        return fetch_entry_count;
    }

    auto fetched_entries_ref() const noexcept -> const std::array<OAMEntry, FETCHED_ENTRY>&
    {
        return fetched_entries;
    }

    void reset()
    {
        number_of_sprite_lines = 0;
        freelist_head_ = nullptr;
    }

    void clear()
    {
        Node<OAMEntry> empty_node{OAMEntry{}, nullptr};
        nodes_.fill(empty_node);
    }

   private:
    std::array<Node<OAMEntry>, SIZE> nodes_{};
    Node<OAMEntry>* freelist_head_{nullptr};
    uint8_t number_of_sprite_lines{0};

    uint8_t fetch_entry_count{0};
    std::array<OAMEntry, FETCHED_ENTRY> fetched_entries{};
};

}  // namespace gameboy::ppu