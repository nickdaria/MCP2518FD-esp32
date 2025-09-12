/*
 * Copyright 2025 Circuit Board Medics
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
