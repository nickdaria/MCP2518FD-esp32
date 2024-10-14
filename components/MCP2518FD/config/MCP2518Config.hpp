/*
 * Copyright 2024 Circuit Board Medics
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
#include <cstddef>
#include <cstdint>

namespace cbm {

static constexpr int cOscillatorFreqMHz = 40;
static constexpr int cOscillatorFreqHz = cOscillatorFreqMHz * 1000 * 1000;
static constexpr int cSysClockDivider = 1;

static constexpr bool cEnableCANFD = false;
static constexpr int cDefaultNominalBitrate = 500 * 1000;
static constexpr int cDefaultDataBitrate = 2 * 1000 * 1000;

static constexpr size_t cTEFSize = 0;
static constexpr size_t cTXQSize = 0;
static constexpr size_t cTXQPayloadSize = 0;

static constexpr std::array cTxFIFOSizes = { 32 };
static constexpr std::array cTxFIFOPayloadSizes = { 8 };
static constexpr size_t cNumTxFIFOs = cTxFIFOSizes.size();
// One TX FIFO is tracked via interrupts
static constexpr uint8_t cTxInterruptFIFO = 0;

static constexpr std::array cRxFIFOSizes = { 32 };
static constexpr std::array cRxFIFOPayloadSizes = { 8 };
static constexpr size_t cNumRxFIFOs = cRxFIFOSizes.size();
// One RX FIFO is tracked via interrupts
static constexpr uint8_t cRxInterruptFIFO = 0;

static constexpr size_t cNumFIFOs = cNumTxFIFOs + cNumRxFIFOs;

static_assert(cTxFIFOSizes.size() == cTxFIFOPayloadSizes.size());
static_assert(cRxFIFOSizes.size() == cRxFIFOPayloadSizes.size());
static_assert(cDefaultNominalBitrate <= 1000000);
static_assert(cDefaultDataBitrate <= 8000000);

} // namesapce cbm
