#pragma once

#include <array>
#include <cstdint>
#include <vector>

namespace cbm {

struct can_frame_t {
    static constexpr uint8_t cMaxCanLength = 8;

    uint32_t standard_id       : 11;
    uint32_t extended_id       : 18;
    uint32_t error_flag        : 1;
    uint32_t rtr               : 1;
    uint32_t frame_format_flag : 1;

    std::array<uint8_t, cMaxCanLength> data;

    uint8_t dlc;
};

struct can_fd_frame_t {
    uint32_t standard_id       : 11;
    uint32_t sid_11            : 1;
    uint32_t extended_id       : 18;
    uint32_t frame_format_flag : 1;

    std::vector<uint8_t> data;

    uint8_t dlc;

    explicit can_fd_frame_t(uint8_t payload_size) : data(payload_size) {}
};

} // namespace cbm
