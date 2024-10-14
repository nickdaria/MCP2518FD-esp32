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

#include <algorithm>
#include <cstring>
#include <memory>

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "esp_err.h"
#include "esp_heap_caps.h"

#include "MCP2518.hpp"
#include "CAN.hpp"

#ifdef CONFIG_MCP2518_ATTR
#define MCP2518_FAST_PATH IRAM_ATTR
#else
#define MCP2518_FAST_PATH
#endif

#ifdef CONFIG_MCP2518_CRC_ENABLE
#include "MCP2518CRC.hpp"
#endif

#define WRITE_REGISTER(reg) writeRegister(reg.address, reg.bits.to_ulong())

namespace cbm {

namespace {

extern "C" void MCP2518_FAST_PATH mcp2518__rx_isr(void* ctx)
{
    static_cast<MCP2518*>(ctx)->rxIsr();
}

extern "C" void MCP2518_FAST_PATH mcp2518__tx_isr(void* ctx)
{
    static_cast<MCP2518*>(ctx)->txIsr();
}

} // anonymous namespace

void MCP2518::begin(const Config& config)
{
    _rx_intr_pin = config.rx_intr_pin;
    _tx_intr_pin = config.tx_intr_pin;

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };

#ifdef CONFIG_MCP2518_INSTALL_GPIO_ISR
    if (!_isr_service_installed) {
        // GPIO ISR service can only be installed once
        // This check will prevent MCP2518s from trying to install it twice,
        // but unrelated modules may have already installed the service,
        // causing this to print an error
        _isr_service_installed = true;
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL2);
    }
#endif

    if (_rx_intr_pin != GPIO_NUM_NC) {
        gpio_cfg.pin_bit_mask = BIT64(_rx_intr_pin);
        ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
        ESP_ERROR_CHECK(gpio_isr_handler_add(_rx_intr_pin, mcp2518__rx_isr, this));
    }
    if (_tx_intr_pin != GPIO_NUM_NC) {
        gpio_cfg.pin_bit_mask = BIT64(_tx_intr_pin);
        ESP_ERROR_CHECK(gpio_config(&gpio_cfg));
        ESP_ERROR_CHECK(gpio_isr_handler_add(_tx_intr_pin, mcp2518__tx_isr, this));
    }

    spi_device_interface_config_t spi_cfg = {
        .command_bits = cNumCommandBits,
        .address_bits = cNumAddressBits,
        .dummy_bits = 0,
        .mode = 0,
        .clock_source = SPI_CLK_SRC_DEFAULT,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = cSpiClockHz,
        .input_delay_ns = 0,
        .spics_io_num = config.cs_pin,
        .flags = 0,
        .queue_size = 4,
        .pre_cb = nullptr,
        .post_cb = nullptr
    };
    ESP_ERROR_CHECK(spi_bus_add_device(config.spi_host, &spi_cfg, &_spi));

    // DMA buffers must be 32-bit aligned per ESP
    _tx_buffer = heap_caps_aligned_alloc(4, cMaxTxPayloadSize, MALLOC_CAP_DMA);
    _rx_buffer = heap_caps_aligned_alloc(4, cMaxRxPayloadSize, MALLOC_CAP_DMA);

    MCP2518_ASSERT(_tx_buffer && _rx_buffer);

    memset(_tx_buffer, 0, cMaxTxPayloadSize);

    configDefault();
}

void MCP2518::SetNominalBitrate(int32_t bitrate)
{
    SetMode(Mode::CONFIG);

    BitTiming bt { cSysClockHz, bitrate, true };
    Registers::C1NBTCFG reg { bt };
    WRITE_REGISTER(reg);

    SetMode(Mode::NORMAL);
}

void MCP2518::SetDataBitrate(int32_t bitrate)
{
    SetMode(Mode::CONFIG);

    BitTiming bt { cSysClockHz, bitrate, false };
    Registers::C1DBTCFG reg { bt };
    WRITE_REGISTER(reg);

    SetMode(Mode::NORMAL);
}

void MCP2518::SetMode(Mode mode)
{
    MCP2518_DEBUG_PRINT("Setting mode to %u", static_cast<unsigned>(mode));
    Registers::C1CON reg { readRegister(Registers::C1CON::address) };
    reg.REQOP = std::to_underlying(mode);
    WRITE_REGISTER(reg);
}

MCP2518::Status MCP2518_FAST_PATH MCP2518::ReadMessage(uint8_t fifo_index, can_frame_t& frame, uint32_t& timestamp)
{
    if (!IsRxAvailable(fifo_index)) {
        return Status::NO_MSG;
    }
    readMessageInternal(fifo_index, frame, timestamp);
    return Status::SUCCESS;
}

MCP2518::Status MCP2518_FAST_PATH MCP2518::ReadMessage(uint8_t fifo_index, can_fd_frame_t& frame, uint32_t& timestamp)
{
    if (!IsRxAvailable(fifo_index)) {
        return Status::NO_MSG;
    }
    readMessageInternal(fifo_index, frame, timestamp);
    return Status::SUCCESS;
}

MCP2518::Status MCP2518_FAST_PATH MCP2518::SendMessage(uint8_t tx_fifo_index, const can_frame_t& frame)
{
    if (auto status = QueueMessage(tx_fifo_index, frame); status != Status::SUCCESS) {
        return status;
    }
    SetTxRTS(tx_fifo_index);
    return Status::SUCCESS;
}

MCP2518::Status MCP2518_FAST_PATH MCP2518::SendMessage(uint8_t tx_fifo_index, const can_fd_frame_t& frame)
{
    if (auto status = QueueMessage(tx_fifo_index, frame); status != Status::SUCCESS) {
        return status;
    }
    SetTxRTS(tx_fifo_index);
    return Status::SUCCESS;
}

MCP2518::Status MCP2518_FAST_PATH MCP2518::QueueMessage(uint8_t tx_fifo_index, const can_frame_t& frame)
{
    if (!IsTxAvailable(tx_fifo_index)) {
        return Status::TX_ALL_BUSY;
    }

    auto msg = static_cast<TXMessage<>*>(_tx_buffer);

    msg->header.SID = frame.standard_id;
    msg->header.EID = frame.extended_id;
    msg->header.DLC = frame.dlc;
    msg->header.IDE = frame.frame_format_flag;
    msg->header.RTR = frame.rtr;
    msg->header.BRS = 0;
    msg->header.FDF = 0;

    auto aligned_dst = std::assume_aligned<sizeof(uint32_t)>(msg->data.data());
    auto aligned_src = std::assume_aligned<sizeof(uint32_t)>(static_cast<const void*>(frame.data.data()));
    memcpy(aligned_dst, aligned_src, can_frame_t::cMaxCanLength);

    spi_transaction_t t = {
        .flags = 0,
        .cmd = std::to_underlying(SPICommand::WRITE),
        .addr = getTxFifoAddress(tx_fifo_index),
        .length = (sizeof(TXMessage<>::Header) + cTxFIFOPayloadSizes[tx_fifo_index]) * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = _tx_buffer,
        .rx_buffer = nullptr,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));

    // increment locally
    if (++_tx_fifos_indices[tx_fifo_index] == cTxFIFOSizes[tx_fifo_index]) {
        _tx_fifos_indices[tx_fifo_index] = 0;
    }
    return Status::SUCCESS;
}

MCP2518::Status MCP2518_FAST_PATH MCP2518::QueueMessage(uint8_t tx_fifo_index, const can_fd_frame_t& frame)
{
    if (!IsTxAvailable(tx_fifo_index)) {
        return Status::TX_ALL_BUSY;
    }
    MCP2518_ASSERT(cTxFIFOPayloadSizes[tx_fifo_index] == frame.data.size());

    auto msg = static_cast<TXMessage<>*>(_tx_buffer);

    msg->header.SID = frame.standard_id;
    msg->header.EID = frame.extended_id;
    msg->header.SID11 = frame.sid_11;
    msg->header.DLC = frame.dlc;
    msg->header.IDE = frame.frame_format_flag;
    msg->header.RTR = 0;
    msg->header.BRS = 1;
    msg->header.FDF = 1;

    auto aligned_dst = std::assume_aligned<sizeof(uint32_t)>(msg->data.data());
    auto aligned_src = std::assume_aligned<sizeof(uint32_t)>(static_cast<const void*>(frame.data.data()));
    memcpy(aligned_dst, aligned_src, frame.dlc);

    spi_transaction_t t = {
        .flags = 0,
        .cmd = std::to_underlying(SPICommand::WRITE),
        .addr = getTxFifoAddress(tx_fifo_index),
        .length = (sizeof(TXMessage<>::Header) + cTxFIFOPayloadSizes[tx_fifo_index]) * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = _tx_buffer,
        .rx_buffer = nullptr,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));

    // increment locally
    if (++_tx_fifos_indices[tx_fifo_index] == cTxFIFOSizes[tx_fifo_index]) {
        _tx_fifos_indices[tx_fifo_index] = 0;
    }
    return Status::SUCCESS;
}

void MCP2518_FAST_PATH MCP2518::SetTxRTS(uint8_t tx_fifo_index)
{
    // increment on MCP
    _fifo_configs[tx_fifo_index].UINC = 1;
    _fifo_configs[tx_fifo_index].TXREQ = 1;
    writeRegister(_fifo_configs[tx_fifo_index].address,
                  _fifo_configs[tx_fifo_index].bits.to_ulong());
}

bool MCP2518_FAST_PATH MCP2518::IsRxAvailable(uint8_t fifo_index) const
{
    if (_rx_intr_pin != GPIO_NUM_NC && fifo_index == cRxInterruptFIFO) {
        return !_rx_fifo_not_empty;
    }
    Registers::C1RXIF reg { readRegister(reg.address) };
    return reg.IsInterruptPending(fifo_index + cNumTxFIFOs);
}

bool MCP2518_FAST_PATH MCP2518::IsTxAvailable(uint8_t fifo_index) const
{
    if (_tx_intr_pin != GPIO_NUM_NC && fifo_index == cTxInterruptFIFO) {
        return !_tx_fifo_full;
    }
    Registers::C1TXIF reg { readRegister(reg.address) };
    return !reg.IsInterruptPending(fifo_index);
}

bool MCP2518::IsCANError() const
{
    Registers::C1INT reg { readRegister(reg.address) };
    bool error = reg.IVMIF || reg.CERRIF;
    if (reg.IVMIF) {
        Registers::C1BDIAG1 diag_reg { readRegister(diag_reg.address) };
        MCP2518_DEBUG_PRINT("Invalid message, C1BDIAG1=0x%lx", diag_reg.bits.to_ulong());
        diag_reg.bits &= 0;
        WRITE_REGISTER(diag_reg);
    }
    reg.CERRIF = 0;
    reg.IVMIF = 0;
    WRITE_REGISTER(reg);
    return error;
}

void MCP2518::SetFilter(const Filter& filter, uint8_t filter_index, uint8_t rx_fifo_index)
{
    Registers::C1FLTCON control_reg(filter_index / 4, 0);

    // Each filter control register is shared by 4 filters. Thus the
    // current state of the register needs to be maintained
    control_reg.bits = readRegister(control_reg.address);

    // Disable in order to modify filter and mask
    control_reg.FLTCON[filter_index % 4].FLTEN = 0;
    WRITE_REGISTER(control_reg);

    Registers::C1FLTOBJ obj_reg(filter_index, filter.filter_bits);
    Registers::C1MASK mask_reg(filter_index, filter.mask);
    WRITE_REGISTER(obj_reg);
    WRITE_REGISTER(mask_reg);

    control_reg.FLTCON[filter_index % 4].FBP = 1 + rx_fifo_index + cNumTxFIFOs;
    control_reg.FLTCON[filter_index % 4].FLTEN = 1;
    WRITE_REGISTER(control_reg);
}

void MCP2518::DisableFilter(uint8_t filter_index)
{
    Registers::C1FLTCON reg(filter_index / 4, 0);

    reg.bits = readRegister(reg.address);
    reg.FLTCON[filter_index % 4].FLTEN = 0;

    WRITE_REGISTER(reg);
}

void MCP2518::configDefault()
{
    MCP2518_DEBUG_PRINT("Default configuration");
    SetMode(Mode::CONFIG);

    // Default settings, disable CRC + ECC interrupts
    writeRegister(Registers::OSC::address, 0);
    writeRegister(Registers::IOCON::address, 0);
    writeRegister(Registers::CRC::address, 0);
    writeRegister(Registers::ECCCON::address, 0);

    Registers::C1CON cfg_reg(0);
    cfg_reg.REQOP = std::to_underlying(Mode::CONFIG);
    if constexpr (cTXQSize) {
        cfg_reg.TXQEN = 1;
    }
    if constexpr (cTEFSize) {
        cfg_reg.STEF = 1;
    }
    WRITE_REGISTER(cfg_reg);

    constexpr Registers::C1NBTCFG bit_timing_reg { { cSysClockHz, cDefaultNominalBitrate, true } };
    MCP2518_DEBUG_PRINT("Setting nominal bitrate register to 0x%lx", bit_timing_reg.bits.to_ulong());
    WRITE_REGISTER(bit_timing_reg);

    constexpr Registers::C1DBTCFG data_bit_timing_reg { { cSysClockHz, cDefaultDataBitrate, false } };
    WRITE_REGISTER(data_bit_timing_reg);

    Registers::C1TSCON ts_cfg(0);
    ts_cfg.TBCEN = 1;
    // Make prescalar so that TBC increments every microsecond
    constexpr uint16_t tbc_prescalar = cSysClockHz / 1000 / 1000;
    static_assert((cSysClockHz % 1000 == 0) && ((cSysClockHz / 1000) % 1000 == 0),
        "Clock does not evenly divide to microsecond granularity");
    static_assert(tbc_prescalar, "Clock is too slow for microsecond granularity");
    ts_cfg.TBCPRE = tbc_prescalar - 1;
    WRITE_REGISTER(ts_cfg);

    Registers::C1INT int_cfg(0);
    int_cfg.RXIE = 1;
    int_cfg.TXIE = 1;
    int_cfg.CERRIE = 1;
    WRITE_REGISTER(int_cfg);

    Registers::C1TEFCON tef_cfg(0);
    if constexpr (cTEFSize) {
        tef_cfg.FSIZE = cTEFSize - 1;
        tef_cfg.TEFTSEN = 1;
        tef_cfg.FRESET = 1;
    }
    WRITE_REGISTER(tef_cfg);

    Registers::C1TXQCON txq_cfg(0);
    if constexpr (cTXQSize) {
        constexpr auto payload_size_it = std::find(cPayloadSizes.begin(), cPayloadSizes.end(), cTXQPayloadSize);
        static_assert(cTXQSize == 0 || payload_size_it != cPayloadSizes.end(), "Payload size not supported");
        txq_cfg.PLSIZE = *payload_size_it;
        txq_cfg.FSIZE = cTXQSize - 1;
        txq_cfg.TXQNIE = 1;
        txq_cfg.FRESET = 1;
    }
    WRITE_REGISTER(txq_cfg);

    for (auto& reg : _fifo_configs) {
        WRITE_REGISTER(reg);
        reg.FRESET = 0;
    }

    SetPassthroughFilter();

    SetMode(Mode::NORMAL);
}

#ifdef CONFIG_MCP2518_CRC_ENABLE

uint32_t MCP2518::readRegister(address_t address) const
{
    spi_transaction_t t = {
        .flags = 0,
        .cmd = std::to_underlying(SPICommand::READ_CRC),
        .addr = address,
        // 1 byte for length, 4 bytes for register, 2 for CRC
        .length = (sizeof(uint8_t) + sizeof(uint32_t) + sizeof(uint16_t)) * 8,
        .rxlength = (sizeof(uint32_t) + sizeof(uint16_t)) * 8,
        .user = nullptr,
        .tx_buffer = _tx_buffer,
        .rx_buffer = _rx_buffer,
    };
    *static_cast<uint8_t*>(_tx_buffer) = sizeof(uint32_t);

    spi_device_polling_transmit(_spi, &t);

    constexpr size_t cCrcBufferSize = 7;
    std::array<uint8_t, cCrcBufferSize> crc_buffer = {
        static_cast<uint8_t>((std::to_underlying(SPICommand::READ_CRC) << 4) | ((address >> 8)  & 0xF)),
        static_cast<uint8_t>(address),
        sizeof(uint32_t)
    };
    memcpy(&crc_buffer[3], static_cast<uint8_t*>(_rx_buffer) + 1, sizeof(uint32_t));
    uint16_t crc = calculate_crc(crc_buffer);

    // Read CRC is big-endian for some reason
    uint8_t* crc_ptr = static_cast<uint8_t*>(_rx_buffer) + 1 + sizeof(uint32_t);
    uint16_t read_crc = (crc_ptr[0] << 8) | crc_ptr[1];

    if (crc != read_crc) {
        ESP_LOGE(TAG, "CRC mismatch, read 0x%x, calculated 0x%x", read_crc, crc);
    }
    return *reinterpret_cast<uint32_t*>(static_cast<uint8_t*>(_rx_buffer) + 1);
}

void MCP2518_FAST_PATH MCP2518::writeRegister(address_t address, uint32_t value) const
{
    constexpr size_t cCrcBufferSize = 7;
    std::array<uint8_t, cCrcBufferSize> crc_buffer = {
        static_cast<uint8_t>((std::to_underlying(SPICommand::WRITE_CRC) << 4) | ((address >> 8)  & 0xF)),
        static_cast<uint8_t>(address),
        sizeof(uint32_t)
    };
    *reinterpret_cast<uint32_t*>(&crc_buffer[3]) = value;
    uint16_t crc = calculate_crc(crc_buffer);

    spi_transaction_t t = {
        .flags = 0,
        .cmd = std::to_underlying(SPICommand::WRITE_CRC),
        .addr = address,
        .length = (sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint16_t)) * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = _tx_buffer,
        .rx_buffer = nullptr,
    };
    memcpy(_tx_buffer, &crc_buffer[2], sizeof(uint32_t) + sizeof(uint8_t) + sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));

    Registers::CRC crc_reg { readRegister(crc_reg.address) };
    if (crc_reg.CRCVAL != crc) {
        ESP_LOGE(TAG, "CRC mismatch, read 0x%x, calculated 0x%x", crc_reg.CRCVAL, crc);
    }
}

#else

uint32_t MCP2518_FAST_PATH MCP2518::readRegister(address_t address) const
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA,
        .cmd = std::to_underlying(SPICommand::READ),
        .addr = address,
        .length = sizeof(uint32_t) * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));
    return *(uint32_t*)t.rx_data;
}

void MCP2518_FAST_PATH MCP2518::writeRegister(address_t address, uint32_t value) const
{
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_TXDATA,
        .cmd = std::to_underlying(SPICommand::WRITE),
        .addr = address,
        .length = sizeof(uint32_t) * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = nullptr,
    };
    *(uint32_t*)(&t.tx_data) = value;
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));
}

#endif // CONFIG_MCP2518_CRC_ENABLE

void MCP2518_FAST_PATH MCP2518::readMessageInternal(uint8_t rx_fifo_index, can_frame_t& frame, uint32_t& timestamp)
{
    address_t rxFifoReadAddress = getRxFifoAddress(rx_fifo_index);
    MCP2518_DEBUG_PRINT("Reading from address 0x%x", rxFifoReadAddress);
    spi_transaction_t t = {
        .flags = 0,
        .cmd = std::to_underlying(SPICommand::READ),
        .addr = rxFifoReadAddress,
        .length = (sizeof(RXMessage<>::Header) + cRxFIFOPayloadSizes[rx_fifo_index])  * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = _rx_buffer,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));

    RXMessage<>::Header header;
    memcpy(&header, _rx_buffer, sizeof(header));

    void* aligned_dst = std::assume_aligned<sizeof(uint32_t)>(frame.data.data());
    void* aligned_src = std::assume_aligned<sizeof(uint32_t)>(static_cast<uint8_t*>(_rx_buffer) + sizeof(header));
    memcpy(aligned_dst, aligned_src, can_frame_t::cMaxCanLength);

    frame.dlc               = header.DLC;
    frame.standard_id       = header.SID;
    frame.extended_id       = header.EID;
    frame.error_flag        = header.ESI;
    frame.rtr               = header.RTR;
    frame.frame_format_flag = header.IDE;

    timestamp = header.RXMSGTS;

    incrementRxFifoAddress(rx_fifo_index);
}

void MCP2518_FAST_PATH MCP2518::readMessageInternal(uint8_t rx_fifo_index, can_fd_frame_t& frame, uint32_t& timestamp)
{
    MCP2518_ASSERT(cRxFIFOPayloadSizes[rx_fifo_index] == frame.data.size());
    address_t rxFifoReadAddress = getRxFifoAddress(rx_fifo_index);
    MCP2518_DEBUG_PRINT("Reading from address 0x%x", rxFifoReadAddress);
    spi_transaction_t t = {
        .flags = 0,
        .cmd = std::to_underlying(SPICommand::READ),
        .addr = rxFifoReadAddress,
        .length = (sizeof(RXMessage<>::Header) + cRxFIFOPayloadSizes[rx_fifo_index])  * 8,
        .rxlength = 0,
        .user = nullptr,
        .tx_buffer = nullptr,
        .rx_buffer = _rx_buffer,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(_spi, &t));

    RXMessage<>::Header header;
    memcpy(&header, _rx_buffer, sizeof(header));

    void* aligned_dst = std::assume_aligned<sizeof(uint32_t)>(frame.data.data());
    void* aligned_src = std::assume_aligned<sizeof(uint32_t)>(static_cast<uint8_t*>(_rx_buffer) + sizeof(header));
    memcpy(aligned_dst, aligned_src, cRxFIFOPayloadSizes[rx_fifo_index]);

    frame.dlc               = header.DLC;
    frame.standard_id       = header.SID;
    frame.extended_id       = header.EID;
    frame.sid_11            = header.SID11;
    frame.frame_format_flag = header.IDE;

    timestamp = header.RXMSGTS;

    incrementRxFifoAddress(rx_fifo_index);
}

} // namespace cbm
