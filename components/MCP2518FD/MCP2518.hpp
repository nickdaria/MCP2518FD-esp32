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

#include <algorithm>
#include <array>
#include <bitset>
#include <cstdint>
#include <utility>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"

#include "CAN.hpp"
#include "config/MCP2518Config.hpp"

namespace cbm {

#ifdef CONFIG_MCP2518_ASSERT
#define MCP2518_ASSERT(...) assert(__VA_ARGS__)
#else
#define MCP2518_ASSERT(...)                 \
    do {                                    \
        if (std::is_constant_evaluated()) { \
            assert(__VA_ARGS__);            \
        }                                   \
    } while(0)
#endif

#ifdef CONFIG_MCP2518_DEBUG_PRINT
#define MCP2518_DEBUG_PRINT(...) ESP_LOGI(TAG, __VA_ARGS__)
#else
#define MCP2518_DEBUG_PRINT(...)
#endif

template <typename T, size_t N, size_t... I>
constexpr auto create_array_impl(std::index_sequence<I...>)
{
    return std::array<T, N> { { I... } };
}

/**
 * @brief Create a std array of T types constructed with an index sequence
 * 
 * @tparam T type that comprises the array
 * @tparam N size of the array
 * @return std::array of type T, length N
 */
template <typename T, size_t N>
constexpr auto create_array()
{
    return create_array_impl<T, N>(std::make_index_sequence<N> {});
}

class MCP2518 {
public:
    using address_t = uint16_t;
    using rx_callback_t = void(*)(MCP2518&, void* user_data);
    using tx_callback_t = void(*)(MCP2518&);

    /******************************
     *  Public structs and enums  *
     ******************************/

    struct Config {
        spi_host_device_t spi_host;
        gpio_num_t cs_pin;
        // Transmit interrupt, INT0 pin on MCP2518, set to `GPIO_NUM_NC` if not used
        gpio_num_t tx_intr_pin;
        // Receive interrupt, INT1 pin on MCP2518, set to `GPIO_NUM_NC` if not used
        gpio_num_t rx_intr_pin;
    };

    enum class Status : int_fast8_t { SUCCESS, NO_MSG, TX_ALL_BUSY, idk };

    enum class Mode : uint8_t {
        // In this mode, the device will be on the CAN bus. It can transmit and receive messages in CAN
        // FD mode; bit rate switching can be enabled and up to 64 data bytes can be transmitted and
        // received.
        NORMAL,
        // Sleep mode is a low-power mode, where register and RAM contents are preserved and the
        // clock is switched off
        SLEEP,
        // Loopback mode is a variant of Normal CAN FD Operation mode. This mode will allow internal
        // transmission of messages from the Transmit FIFOs to the Receive FIFOs. The module does not
        // require an external Acknowledge from the bus. No messages can be received from the bus
        // because the RXCAN pin is disconnected
        // The transmit signal is internally connected to receive and TXCAN is driven high
        INTERNAL_LOOPBACK,
        // Listen Only mode is a variant of Normal CAN FD Operation mode. If the Listen Only mode is
        // activated, the module on the CAN bus is passive. It will receive messages, but it will not transmit
        // any bits. TXREQ bits will be ignored. No error flags or Acknowledge signals are sent. The error
        // counters are deactivated in this state. The Listen Only mode can be used for detecting the baud
        // rate on the CAN bus. It is necessary that there are at least two further nodes that communicate
        // with each other. The baud rate can be detected empirically by testing different values until a
        // message is received successfully. This mode is also useful for monitoring the CAN bus without
        // influencing it
        LISTEN_ONLY,
        // After Reset, the CAN FD Controller module is in Configuration mode. The error counters are
        // cleared and all registers contain the Reset values
        CONFIG,
        // The transmit signal is internally connected to receive and transmit messages can be monitored
        // on the TXCAN pin
        EXTERNAL_LOOPBACK,
        // In this mode, the device will be on the CAN bus. This is the Classic CAN 2.0 mode. The module
        // will not receive CAN FD frames. It might send error frames if CAN FD frames are detected on
        // the bus. The FDF, BRS and ESI bits in the TX Objects will be ignored and transmitted as ‘0’.
        NORMAL_CAN_2_0,
        // In Restricted Operation mode, the node is able to receive data and remote frames, and to
        // Acknowledge valid frames, but it does not send data frames, remote frames, error frames or
        // overload frames. In case of an error condition or overload condition, it does not send dominant
        // bits; instead, it waits for the occurrence of the bus Idle condition to resynchronize itself to the
        // CAN communication. The error counters are not incremented
        RESTRICTED_OPERATION,
    };

    struct Filter {
        union {
            struct {
                uint32_t standard_id_mask    : 11;
                uint32_t extended_id_mask    : 18;
                uint32_t standard_id_11_mask : 1;
                uint32_t exide_mask          : 1;
            };
            uint32_t mask;
        };
        union {
            struct {
                uint32_t standard_id    : 11;
                uint32_t extended_id    : 18;
                uint32_t standard_id_11 : 1;
                uint32_t exide          : 1;
            };
            uint32_t filter_bits;
        };
    };

    /**********************
     *  Public functions  *
     **********************/

    consteval MCP2518()
        : _tef_base_address(GetTEFBaseAddress(cMessageRamBaseAddress))
        , _txq_base_address(GetTXQBaseAddress(_tef_base_address))
        , _tx_fifos_base_addresses(GetTxFIFOsBaseAddresses(_txq_base_address))
        , _rx_fifos_base_addresses(GetRxFIFOsBaseAddresses(_tx_fifos_base_addresses.back()))
        , _fifo_configs(Registers::getFifoConfigs())
        , _tx_fifo_full()
        , _rx_fifo_not_empty()
    {
    }

    void begin(const Config&);

    /**
     * @brief Set the Nominal Bitrate
     *
     *        - In CAN 2.0, this is simply the bitrate, as there is no bitrate switching
     *        between arbitration and data transmission.
     *
     *        - In CAN FD, this is the bitrate during arbitration
     * 
     * @param bitrate in bits per second
     */
    void SetNominalBitrate(int32_t bitrate);

    int32_t GetNominalBitrate() { return _nominalBitrate; }

    int32_t GetBaudrate() { return GetNominalBitrate(); }

    int32_t GetDataBitrate() { return _dataBitrate; }

    /**
     * @brief Set the Data Bitrate
     *
     *        - In CAN FD, this is the bitrate during data transmission, after the BRS (bit rate switch).
     *
     *        - In CAN 2.0, or if BRS=0 this has no effect
     * 
     * @param bitrate in bits per second
     */
    void SetDataBitrate(int32_t bitrate);

    /**
     * @brief Set the Mode
     * 
     * @param mode See the `Mode` type for details
     */
    void SetMode(Mode mode);

    /**
     * @brief Checks that a message is available, then reads from the RX FIFO if so
     * 
     * @param frame Output. CAN 2.0 or FD frame
     * @return Status `SUCCESS` or `NO_MSG` if no message is available
     */
    template<typename Frame_t>
    Status ReadMessage(Frame_t& frame)
    {
        uint32_t _;
        return ReadMessage(0, frame, _);
    }

    /**
     * @brief Checks that a message is available, then reads from the RX FIFO if so
     * 
     * @param rx_fifo_index RX FIFO from which to read message
     * @param frame Output. CAN 2.0 or FD frame
     * @return Status `SUCCESS` or `NO_MSG` if no message is available
     */
    template<typename Frame_t>
    Status ReadMessage(uint8_t rx_fifo_index, Frame_t& frame)
    {
        uint32_t _;
        return ReadMessage(0, frame, _);
    }

    /**
     * @brief Checks that a message is available, then reads from the RX FIFO if so
     * 
     * @param frame Output. CAN 2.0 or FD frame
     * @param timestamp Output. In microseconds, unchanged if `NO_MSG`
     * @return Status `SUCCESS` or `NO_MSG` if no message is available
     * 
     * @note The timestamp is according to MCP2518's internal clock and is not
     *       related to any timer on the ESP
     */
    template<typename Frame_t>
    Status ReadMessage(Frame_t& frame, uint32_t& timestamp)
    {
        return ReadMessage(0, frame, timestamp);
    }

    /**
     * @brief Checks that a message is available, then reads from the RX FIFO if so
     * 
     * @param rx_fifo_index RX FIFO from which to read message
     * @param frame Output. CAN 2.0 frame
     * @param timestamp Output. In microseconds, unchanged if `NO_MSG`
     * @return Status `SUCCESS` or `NO_MSG` if no message is available
     * 
     * @note The timestamp is according to MCP2518's internal clock and is not
     *       related to any timer on the ESP
     */
    Status ReadMessage(uint8_t fifo_index, can_frame_t& frame, uint32_t& timestamp);

    /**
     * @brief Checks that a message is available, then reads from the RX FIFO if so
     * 
     * @param rx_fifo_index RX FIFO from which to read message
     * @param frame Output. CAN FD frame
     * @param timestamp Output. In microseconds, unchanged if `NO_MSG`
     * @return Status `SUCCESS` or `NO_MSG` if no message is available
     * 
     * @note The timestamp is according to MCP2518's internal clock and is not
     *       related to any timer on the ESP
     */
    Status ReadMessage(uint8_t fifo_index, can_fd_frame_t& frame, uint32_t& timestamp);

    /**
     * @brief Sends CAN 2.0 frame by appending to TX FIFO tail
     * 
     * @param tx_fifo_index Index of TX FIFO to use, 0-30
     * @param frame CAN 2.0 frame to be sent
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status SendMessage(uint8_t tx_fifo_index, const can_frame_t& frame);

    /**
     * @brief Sends CAN FD frame by appending to TX FIFO tail
     *
     * @param tx_fifo_index Index of TX FIFO to use, 0-30
     * @param frame CAN FD frame to be sent
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status SendMessage(uint8_t tx_fifo_index, const can_fd_frame_t& frame);

    /**
     * @brief Sends CAN 2.0 frame by appending to TX FIFO tail
     *
     * @param frame CAN 2.0 frame to be sent
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status SendMessage(const can_frame_t& frame)
    {
        return SendMessage(0, frame);
    }

    /**
     * @brief Sends CAN FD frame by appending to TX FIFO tail
     *
     * @param frame CAN FD frame to be sent
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status SendMessage(const can_fd_frame_t& frame)
    {
        return SendMessage(0, frame);
    }

    /**
     * @brief Queues CAN 2.0 frame without setting RTS
     *
     * @param tx_fifo_index Which TX FIFO, 0-30
     * @param frame CAN 2.0 frame
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status QueueMessage(uint8_t tx_fifo_index, const can_frame_t& frame);

    /**
     * @brief Queues CAN FD frame without setting RTS
     *
     * @param tx_fifo_index Which TX FIFO, 0-30
     * @param frame CAN FD frame
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status QueueMessage(uint8_t tx_fifo_index, const can_fd_frame_t& frame);

    /**
     * @brief Queues CAN 2.0 frame without setting RTS
     *
     * @param frame CAN 2.0 frame
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status QueueMessage(const can_frame_t& frame)
    {
        return QueueMessage(0, frame);
    }

    /**
     * @brief Queues CAN FD frame without setting RTS
     *
     * @param frame CAN FD frame
     * @return Status `SUCCESS` or `TX_ALL_BUSY` if TX FIFO is full
     */
    Status QueueMessage(const can_fd_frame_t& frame)
    {
        return QueueMessage(0, frame);
    }

    /**
     * @brief Set Tx FIFO ready to send and increment tail pointer on MCP2518
     * 
     * @param tx_fifo_index Which TX FIFO, 0-30
     */
    void SetTxRTS(uint8_t tx_fifo_index = 0);

    /**
     * @brief Set a filter. Messages matching it will be directed to the given RX FIFO
     * 
     * @param filter A combination of IDs and bitmask
     * @param filter_index Which filter to set, 0-31
     * @param rx_fifo_index RX FIFO to which matching messages will be directed
     */
    void SetFilter(const Filter& filter, uint8_t filter_index = 0, uint8_t rx_fifo_index = 0);

    /**
     * @brief Set a filter that accepts all messages
     * 
     * @param filter_index Which filter to modify, 0-31
     * @param rx_fifo_index RX FIFO to which all messages will be directed
     */
    void SetPassthroughFilter(uint8_t filter_index = 0, uint8_t rx_fifo_index = 0)
    {
        Filter f;
        f.mask = 0;
        SetFilter(f, filter_index, rx_fifo_index);
    }

    /**
     * @brief Disables the chosen filter
     * 
     * @param filter_index Which filter to disable, 0-31
     * 
     * @warning At least one filter must be enabled at all times to receive messages.
     *          This can be a filter that accepts all messages, but it must be enabled.
     *          To accept all messages, use `SetPassthroughFilter`
     */
    void DisableFilter(uint8_t filter_index);

    bool IsRxAvailable(uint8_t fifo_index = 0) const;

    bool IsTxAvailable(uint8_t fifo_index = 0) const;

    bool IsCANError() const;

    void rxIsr()
    {
        _rx_fifo_not_empty = !_rx_fifo_not_empty;
        if (_rxCb && _rx_fifo_not_empty) {
            _rxCb(*this, _userData);
        }
    }

    void txIsr()
    {
        _tx_fifo_full = !_tx_fifo_full;
        if (_txCb) {
            _txCb(*this);
        }
    }

    void RegisterRxCallback(rx_callback_t cb, void* user_data)
    {
        _rxCb = cb;
        _userData = user_data;
    }

    void RegisterTxCallback(tx_callback_t cb)
    {
        _txCb = cb;
    }

private:
    /*******************************
     *  Private structs and enums  *
     *******************************/

    enum class InterruptCode : uint8_t {
        TRANSMIT_ATTEMPT        = 0b1001010,
        TRANSMIT_EVENT          = 0b1001001,
        INVALID_MESSAGE         = 0b1001000,
        OPERATION_MODE_CHANGE   = 0b1000111,
        TBC_OVERFLOW            = 0b1000110,
        // RX: message received before previous message was
        // saved to memory; TX: can't feed TX MAB fast enough to transmit consistent data.
        RXTX_MAB_OVERFLOW       = 0b1000101,
        ADDRESS_ERROR           = 0b1000100,
        RX_FIFO                 = 0b1000011,
        WAKEUP                  = 0b1000010,
        ERROR                   = 0b1000001,
        NO_INTERRUPT            = 0b1000000,
        TXQ                     = 0b0000000,
        FIFO_1                  = 0b0000001,
        FIFO_2,
        FIFO_3,
        FIFO_4,
        FIFO_5,
        FIFO_6,
        FIFO_7,
        FIFO_8,
        FIFO_9,
        FIFO_10,
        FIFO_11,
        FIFO_12,
        FIFO_13,
        FIFO_14,
        FIFO_15,
        FIFO_16,
        FIFO_17,
        FIFO_18,
        FIFO_19,
        FIFO_20,
        FIFO_21,
        FIFO_22,
        FIFO_23,
        FIFO_24,
        FIFO_25,
        FIFO_26,
        FIFO_27,
        FIFO_28,
        FIFO_29,
        FIFO_30,
        FIFO_31,
    };

    enum class SPICommand : uint8_t {
        RESET      = 0b0000,
        READ       = 0b0011,
        WRITE      = 0b0010,
        READ_CRC   = 0b1011,
        WRITE_CRC  = 0b1010,
        WRITE_SAFE = 0b1100,
    };

    struct BitTiming {
        // Baud rate prescalar is used to divide the SYSCLK
        int BRP = 1;
        static constexpr int cMaxBRP = 256;
        // Synchronization Segment (SYNC) - Synchronizes the different nodes connected on the CAN
        // bus. A bit edge is expected to be within this segment. The Synchronization Segment is
        // always 1 TQ
        static constexpr int SYNC = 1;
        // Propagation Segment (PRSEG) - Compensates for the propagation delay on the bus. PRSEG
        // has to be longer than the maximum propagation delay
        //
        // @note On MCP2518, this is combined with Phase 1, so there is no reason to calculate a separate value
        int PRSEG = 0;
        // Phase Segment 1 (PHSEG1) - This time segment compensates for errors that may occur due
        // to phase shifts in the edges. The time segment may be automatically lengthened during
        // resynchronization to compensate for the phase shift.
        int PHSEG1 = 1;
        static constexpr int cMaxNTSEG1 = 255;
        static constexpr int cMaxDTSEG1 = 32;
        // Phase Segment 2 (PHSEG2) - This time segment compensates for errors that may occur due
        // to phase shifts in the edges. The time segment may be automatically shortened during
        // resynchronization to compensate for the phase shift.
        int PHSEG2 = 1;
        static constexpr int cMaxNTSEG2 = 128;
        static constexpr int cMaxDTSEG2 = 16;
        // The Synchronization Jump Width (SJW) is the maximum amount PHSEG1 and PHSEG2 can be
        // adjusted during resynchronization
        int SJW = 1;
        static constexpr int cMaxNSJW = 128;
        static constexpr int cMaxDSJW = 16;

        static constexpr int cMaxNTQ = SYNC + cMaxNTSEG1 + cMaxNTSEG2;
        static constexpr int cMaxDTQ = SYNC + cMaxDTSEG1 + cMaxDTSEG2;

        constexpr BitTiming(int sysclock, int bitrate, bool nominal)
        {
            constexpr int target_sample_point = 80;
            const int bit_time_ns = (1000 * 1000 * 1000) / bitrate;
            for (size_t pass = 0; pass < 2; ++pass) {
                BRP = 1;
                // loosen requirements on second pass
                bool best_effort = pass == 1;
                for (; BRP < cMaxBRP; ++BRP) {
                    uint64_t tq = ((1000 * 1000 * 1000) / sysclock * BRP);
                    int tq_per_bit = bit_time_ns / tq;
                    if ((!best_effort && bit_time_ns % tq) ||
                        (nominal && tq_per_bit > cMaxNTQ)  ||
                        (!nominal && tq_per_bit > cMaxDTQ)) {
                        continue;
                    }
                    constexpr int phase_two_percent = 100 - target_sample_point;
                    if (!best_effort && (phase_two_percent * tq_per_bit) % 100) {
                        continue;
                    }
                    PHSEG2 = (phase_two_percent * tq_per_bit) / 100;
                    if ((nominal && (PHSEG2 >= cMaxNTSEG2 || PHSEG2 == 0)) ||
                        (!nominal && (PHSEG2 >= cMaxDTSEG2 || PHSEG2 == 0))) {
                        continue;
                    }
                    PHSEG1 = tq_per_bit - PHSEG2 - SYNC - PRSEG;
                    if ((nominal && (PHSEG1 >= cMaxNTSEG1 || PHSEG1 == 0)) ||
                        (!nominal && (PHSEG1 >= cMaxDTSEG1 || PHSEG1 == 0))) {
                        continue;
                    }
                    SJW = std::min(PHSEG1, PHSEG2);
                    SJW = nominal ? std::min(cMaxNSJW, SJW) : std::min(cMaxDSJW, SJW);
                    return;
                }
            }
            // unreachable ?
            // Fail compilation, but don't crash during runtime
            if (std::is_constant_evaluated()) {
                assert(0 && "Unable to find a bit timing even close");
            }
        }
    };

    struct Registers { // in-class "namespace"

        // OSCILLATOR CONTROL REGISTER
        struct OSC {
            static constexpr address_t address = 0xe00;

            explicit OSC(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // PLL Enable
                    // `1` = System Clock from 10x PLL
                    // `0` = System Clock comes directly from XTAL oscillator
                    uint32_t PLLEN    : 1;
                    uint32_t padding0 : 1;
                    // Clock (Oscillator) Disable
                    // `1` = Clock disabled, the device is in Sleep mode.
                    // `0` = Enable Clock
                    uint32_t OSCDIS   : 1;
                    // Low Power Mode (LPM) Enable
                    // `1` = When in LPM, the device will stop the clock and power down the majority of the chip. Register and
                    // RAM values will be lost. The device will wake-up due to asserting nCS, or due to RXCAN activity.
                    // `0` = When in Sleep mode, the device will stop the clock, and retain it’s reg
                    uint32_t LPMEN    : 1;
                    // System Clock Divisor
                    // `1` = SCLK is divided by 2
                    // `0` = SCLK is divided by 1
                    uint32_t SCLKDIV  : 1;
                    // Clock Output Divisor
                    // `11` = CLKO is divided by 10
                    // `10` = CLKO is divided by 4
                    // `01` = CLKO is divided by 2
                    // `00` = CLKO is divided by 1
                    uint32_t CLKODIV  : 2;
                    uint32_t padding1 : 1;
                    // PLL Ready
                    // `1` = PLL Locked
                    // `0` = PLL not ready
                    uint32_t PLLRDY   : 1;
                    uint32_t padding2 : 1;
                    // Clock Ready
                    // `1` = Clock is running and stable
                    // `0` = Clock not ready or off
                    uint32_t OSCRDY   : 1;
                    uint32_t padding3 : 1;
                    // Synchronized SCLKDIV bit
                    // `1` = SCLKDIV 1
                    // `0` = SCLKDIV 0
                    uint32_t SCLKRDY  : 1;
                    uint32_t padding4 : 19;
                };
                std::bitset<32> bits;
            };
        };
        static_assert(sizeof(OSC) == sizeof(uint32_t));

        // INPUT/OUTPUT CONTROL REGISTER
        // @note The bit fields in the IOCON register must be written using single data byte SFR WRITE instructions
        struct IOCON {
            static constexpr address_t address = 0xe04;

            explicit IOCON(uint32_t bits) : bits(bits) {}

            union {
                struct {
                    // GPIO0 Data Direction
                    // `1` = Input Pin
                    // `0` = Output Pin
                    // @note If PMx = 0, TRISx will be ignored and the pin will be an output.
                    uint32_t TRIS0      : 1;
                    // GPIO1 Data Direction
                    // `1` = Input Pin
                    // `0` = Output Pin
                    // @note If PMx = 0, TRISx will be ignored and the pin will be an output.
                    uint32_t TRIS1      : 1;
                    uint32_t padding0   : 4;
                    // Enable Transceiver Standby Pin Control
                    // `1` = XSTBY control enabled
                    // `0` = XSTBY control disabled
                    uint32_t XSTBYEN    : 1;
                    uint32_t padding1   : 1;
                    // GPIO0 Latch
                    // `1` = Drive Pin High
                    // `0` = Drive Pin Low
                    uint32_t LAT0       : 1;
                    // GPIO1 Latch
                    // `1` = Drive Pin High
                    // `0` = Drive Pin Low
                    uint32_t LAT1       : 1;
                    uint32_t padding2   : 6;
                    // GPIO0 Status
                    // `1` = VGPIO0 > VIH
                    // `0` = VGPIO0 < VIL
                    uint32_t GPIO0      : 1;
                    // GPIO1 Status
                    // `1` = VGPIO1 > VIH
                    // `0` = VGPIO1 < VIL
                    uint32_t GPIO1      : 1;
                    uint32_t padding3   : 6;
                    // GPIO Pin Mode
                    // `1` = Pin is used as GPIO0
                    // `0` = Interrupt Pin INT0, asserted when CiINT.RXIF and RXIE are set
                    uint32_t PM0        : 1;
                    // GPIO Pin Mode
                    // `1` = Pin is used as GPIO1
                    // `0` = Interrupt Pin INT1, asserted when CiINT.RXIF and RXIE are set
                    uint32_t PM1        : 1;
                    uint32_t padding4   : 2;
                    // TXCAN Open Drain Mode
                    // `1` = Open Drain Output
                    // `0` = Push/Pull Output
                    uint32_t TXCANOD    : 1;
                    // Start-Of-Frame signal
                    // `1` = SOF signal on CLKO pin
                    // `0` = Clock on CLKO pin
                    uint32_t SOF        : 1;
                    // Interrupt pins Open Drain Mode
                    // `1` = Open Drain Output
                    // `0` = Push/Pull Output
                    uint32_t INTOD      : 1;
                    uint32_t padding5   : 1;
                };
                std::bitset<32> bits;
            };
        };

        // CRC REGISTER
        struct CRC {
            static constexpr address_t address = 0xe08;

            explicit CRC(uint32_t bits) : bits(bits) {}

            union {
                struct {
                    // Cycle Redundancy Check from last CRC mismatch
                    uint32_t CRCVAL     : 16;
                    // CRC Error Interrupt Flag
                    // `1` = CRC mismatch occurred
                    // `0` = No CRC error has occurred
                    uint32_t CRCERRIF   : 1;
                    // CRC Command Format Error Interrupt Flag
                    // `1` = Number of Bytes mismatch during "SPI with CRC" command occurred
                    // `0` = No SPI CRC command format error occurred
                    uint32_t FERRIF     : 1;
                    uint32_t padding0   : 6;
                    // CRC Error Interrupt Enable
                    uint32_t CRCERRIE   : 1;
                    // CRC Command Format Error Interrupt Enable
                    uint32_t FERRIE     : 1;
                    uint32_t padding1   : 6;
                };
                std::bitset<32> bits;
            };
        };

        // ECC CONTROL REGISTER
        struct ECCCON {
            static constexpr address_t address = 0xe0c;

            explicit ECCCON(uint32_t bits) : bits(bits) {}

            union {
                struct {
                    // ECC Enable
                    // `1` = ECC enabled
                    // `0` = ECC disabled
                    uint32_t ECCEN      : 1;
                    // Single Error Detection Interrupt Enable Flag
                    uint32_t SECIE      : 1;
                    // Double Error Detection Interrupt Enable Flag
                    uint32_t DEDIE      : 1;
                    uint32_t padding0   : 5;
                    // Parity bits used during write to RAM when ECC is disabled
                    uint32_t PARITY     : 7;
                    uint32_t padding1   : 17;
                };
                std::bitset<32> bits;
            };
        };

        // ECC STATUS REGISTER
        struct ECCSTAT {
            static constexpr address_t address = 0xe10;

            explicit ECCSTAT(uint32_t bits) : bits(bits) {}

            union {
                struct {
                    uint32_t padding0   : 1;
                    // Single Error Detection Interrupt Flag
                    // `1` = Single Error was detected
                    // `0` = No Single Error occurred
                    uint32_t SECIF      : 1;
                    // Double Error Detection Interrupt Flag
                    // `1` = Double Error was detected
                    // `0` = No Double Error Detection occurred
                    uint32_t DEDIF      : 1;
                    uint32_t padding1   : 8;
                    // Address where last ECC error occurred
                    uint32_t ERRADDR    : 12;
                    uint32_t padding2   : 4;
                };
                std::bitset<32> bits;
            };
        };

        // DEVICE ID REGISTER
        struct DEVID {
            static constexpr address_t address = 0xe14;

            explicit DEVID(uint32_t bits) : bits(bits) {}

            union {
                struct {
                    // Silicon Revision
                    uint32_t REV        : 4;
                    // Device ID
                    uint32_t ID         : 4;
                    uint32_t padding0   : 24;
                };
                std::bitset<32> bits;
            };
        };

        // CAN CONTROL REGISTER
        struct C1CON {
            static constexpr address_t address = 0x000;

            explicit C1CON(uint32_t bits) : bits(bits) {}

            union {
                struct {
                    // Device Net Filter Bit Number bits
                    // `10011`-`11111` = Invalid Selection (compare up to 18-bits of data with EID)
                    // `10010` = Compare up to data byte 2 bit 6 with EID17
                    // ...
                    // `00001` = Compare up to data byte 0 bit 7 with EID0
                    // `00000` = Do not compare data bytes
                    uint32_t DNCNT      : 5;
                    // Enable ISO CRC in CAN FD Frames bit
                    // `1` = Include Stuff Bit Count in CRC Field and use Non-Zero CRC Initialization Vector according to ISO
                    // 11898-1:2015
                    // `0` = Do NOT include Stuff Bit Count in CRC Field and use CRC Initialization Vector with all zeros
                    uint32_t ISOCRCEN   : 1;
                    // Protocol Exception Event Detection Disabled bit
                    // A recessive “res bit” following a recessive FDF bit is called a Protocol Exception.
                    // `1` = Protocol Exception is treated as a Form Error.
                    // `0` = If a Protocol Exception is detected, the CAN FD Controller module will enter Bus Integrating state
                    uint32_t PXEDIS     : 1;
                    uint32_t padding0   : 1;
                    // Enable CAN Bus Line Wake-up Filter bit
                    // `1` = Use CAN bus line filter for wake-up
                    // `0` = CAN bus line filter is not used for wake-up
                    uint32_t WAKFIL     : 1;
                    // Selectable Wake-up Filter Time bits
                    // `00` = T00FILTER
                    // `01` = T01FILTER
                    // `10` = T10FILTER
                    // `11` = T11FILTER
                    uint32_t WFT        : 2;
                    // CAN Module is Busy bit
                    // `1` = The CAN module is transmitting or receiving a message
                    // `0` = The CAN module is inactive
                    uint32_t BUSY       : 1;
                    // Bit Rate Switching Disable bit
                    // `1` = Bit Rate Switching is Disabled, regardless of BRS in the Transmit Message Object
                    // `0` = Bit Rate Switching depends on BRS in the Transmit Message Object
                    uint32_t BRSDIS     : 1;
                    uint32_t padding1   : 3;
                    // Restrict Retransmission Attempts bit
                    // `1` = Restricted retransmission attempts, CiFIFOCONm.TXAT is used
                    // `0` = Unlimited number of retransmission attempts, CiFIFOCONm.TXAT will be ignored
                    uint32_t RTXAT      : 1;
                    // Transmit ESI in Gateway Mode bit
                    // `1` = ESI is transmitted recessive when ESI of message is high or CAN FD Controller error passive
                    // `0` = ESI reflects error status of CAN FD Controller
                    uint32_t ESIGM      : 1;
                    // Transition to Listen Only Mode on System Error bit
                    // `1` = Transition to Listen Only Mode
                    // `0` = Transition to Restricted Operation Mode
                    uint32_t SERR2LOM   : 1;
                    // Store in Transmit Event FIFO bit
                    // `1` = Saves transmitted messages in TEF and reserves space in RAM
                    // `0` = Do not save transmitted messages in TEF
                    uint32_t STEF       : 1;
                    // Enable Transmit Queue bit(1)
                    // `1` = Enables TXQ and reserves space in RAM
                    // `0` = Do not reserve space in RAM for TXQ
                    uint32_t TXQEN      : 1;
                    // Operation Mode Status bits, see `Mode`
                    uint32_t OPMOD      : 3;
                    // Request Operation Mode bits, see `Mode`
                    uint32_t REQOP      : 3;
                    // Abort All Pending Transmissions bit
                    // `1` = Signal all transmit FIFOs to abort transmission
                    // `0` = Module will clear this bit when all transmissions were aborted
                    uint32_t ABAT       : 1;
                    // Transmit Bandwidth Sharing bits
                    // Delay between two consecutive transmissions (in arbitration bit times)
                    // `0000` = No delay
                    // `0001` = 2
                    // `0010` = 4
                    // `0011` = 8
                    // `0100` = 16
                    // `0101` = 32
                    // `0110` = 64
                    // `0111` = 128
                    // `1000` = 256
                    // `1001` = 512
                    // `1010` = 1024
                    // `1011` = 2048
                    // `1111`-`1100` = 4096
                    uint32_t TXBWS      : 4;
                };
                std::bitset<32> bits;
            };
        };

        // NOMINAL BIT TIME CONFIGURATION REGISTER
        struct C1NBTCFG {
            static constexpr address_t address = 0x004;

            explicit C1NBTCFG(uint32_t bits) : bits(bits) {}

            constexpr C1NBTCFG(BitTiming bt) {
                SJW = bt.SJW - 1;
                padding0 = 0;
                TSEG2 = bt.PHSEG2 - 1;
                padding1 = 0;
                TSEG = (bt.PHSEG1 + bt.PRSEG) - 1;
                BRP = bt.BRP - 1;
            }
            union {
                struct {
                    // Synchronization Jump Width bits
                    // `111 1111` = Length is 128 x TQ
                    // ...
                    // `000 0000` = Length is 1 x TQ
                    uint32_t SJW        : 7;
                    uint32_t padding0   : 1;
                    // Time Segment 2 bits (Phase Segment 2)
                    // `111 1111` = Length is 128 x TQ
                    // ...
                    // `000 0000` = Length is 1 x TQ
                    uint32_t TSEG2      : 7;
                    uint32_t padding1   : 1;
                    // Time Segment 1 bits (Propagation Segment + Phase Segment 1)
                    // `1111 1111` = Length is 256 x TQ
                    // ...
                    // `0000 0000` = Length is 1 x TQ
                    uint32_t TSEG       : 8;
                    // Baud Rate Prescaler bits
                    // `1111 1111` = TQ = 256/Fsys
                    // ...
                    // `0000 0000` = TQ = 1/Fsys
                    uint32_t BRP        : 8;
                };
                std::bitset<32> bits;
            };
        };

        // DATA BIT TIME CONFIGURATION REGISTER
        struct C1DBTCFG {
            static constexpr address_t address = 0x008;

            explicit C1DBTCFG(uint32_t bits) : bits(bits) {}
            constexpr C1DBTCFG(BitTiming bt) {
                SJW = bt.SJW - 1;
                padding0 = 0;
                TSEG2 = bt.PHSEG2 - 1;
                padding1 = 0;
                TSEG = (bt.PHSEG1 + bt.PRSEG) - 1;
                padding2 = 0;
                BRP = bt.BRP - 1;
            }
            union {
                struct {
                    // Synchronization Jump Width bits
                    // `1111` = Length is 16 x TQ
                    // ...
                    // `0000` = Length is 1 x TQ
                    uint32_t SJW        : 4;
                    uint32_t padding0   : 4;
                    // Time Segment 2 bits (Phase Segment 2)
                    // `1111` = Length is 16 x TQ
                    // ...
                    // `0000` = Length is 1 x TQ
                    uint32_t TSEG2      : 4;
                    uint32_t padding1   : 4;
                    // Time Segment 1 bits (Propagation Segment + Phase Segment 1)
                    // `1 1111` = Length is 32 x TQ
                    // ...
                    // `0 0000` = Length is 1 x TQ
                    uint32_t TSEG       : 5;
                    uint32_t padding2   : 3;
                    // Baud Rate Prescaler bits
                    // `1111 1111` = TQ = 256/Fsys
                    // ...
                    // `0000 0000` = TQ = 1/Fsys
                    uint32_t BRP        : 8;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMITTER DELAY COMPENSATION REGISTER
        struct C1TDC {
            static constexpr address_t address = 0x00c;

            explicit C1TDC(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmitter Delay Compensation Value bits; Secondary Sample Point (SSP)
                    // `11 1111` = 63 x TSYSCLK
                    // ...
                    // `00 0000` = 0 x TSYSCLK
                    uint32_t TDCV       : 6;
                    uint32_t padding0   : 2;
                    // Transmitter Delay Compensation Offset bits; Secondary Sample Point (SSP)
                    // Two’s complement; offset can be positive or zero, therefore, bit 14 must always be set to 0.
                    // `11 1111` = 63 x TSYSCLK
                    // ...
                    // `00 0000` = 0 x TSYSCLK
                    uint32_t TDCO       : 6;
                    uint32_t padding1   : 2;
                    // Transmitter Delay Compensation Mode bits; Secondary Sample Point (SSP)
                    // `10`-`11` = Auto; measure delay and add TDCO.
                    // `01` = Manual; Do not measure, use TDCV + TDCO from register
                    // `00` = TDC Disabled
                    uint32_t TDCMOD     : 2;
                    uint32_t padding2   : 6;
                    // Enable 12-Bit SID in CAN FD Base Format Messages bit
                    // `1` = RRS is used as SID11 in CAN FD base format messages: SID[11:0] = {SID[10:0], SID11}
                    // `0` = Do not use RRS; SID[10:0] according to ISO 11898-1:2015
                    uint32_t SID11EN    : 1;
                    // Enable Edge Filtering during Bus Integration state bit
                    // `1` = Edge Filtering enabled, according to ISO 11898-1:2015
                    // `0` = Edge Filtering disabled
                    uint32_t EDGFLTEN   : 1;
                    uint32_t padding3   : 6;
                };
                std::bitset<32> bits;
            };
        };

        // TIME BASE COUNTER REGISTER
        struct C1TBC {
            static constexpr address_t address = 0x010;

            explicit C1TBC(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Time Base Counter bits
                    // This is a free running timer that increments every TBCPRE clocks when TBCEN is set
                    // @note The TBC will be stopped and reset when TBCEN = `0`
                    uint32_t TBC;
                };
                std::bitset<32> bits;
            };
        };

        // TIME STAMP CONTROL REGISTER
        struct C1TSCON {
            static constexpr address_t address = 0x014;

            explicit C1TSCON(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Time Base Counter Prescaler bits
                    // `1023` = TBC increments every 1024 clocks
                    // ...
                    // `0` = TBC increments every 1 clock
                    uint32_t TBCPRE     : 10;
                    uint32_t padding0   : 6;
                    // Time Base Counter Enable bit
                    // `1` = Enable TBC
                    // `0` = Stop and reset TBC
                    uint32_t TBCEN      : 1;
                    // Time Stamp EOF bit
                    // `1` = Time Stamp when frame is taken valid:
                    // - RX no error until last but one bit of EOF
                    // - TX no error until the end of EOF
                    //
                    // `0` = Time Stamp at “beginning” of Frame:
                    // - Classical Frame: at sample point of SOF
                    // - FD Frame: see TSRES bit.
                    uint32_t TSEOF      : 1;
                    // Time Stamp res bit (FD Frames only)
                    // `1` = at sample point of the bit following the FDF bit.
                    // `0` = at sample point of SOF
                    uint32_t TSRES      : 1;
                    uint32_t padding1   : 13;
                };
                std::bitset<32> bits;
            };
        };

        // INTERRUPT CODE REGISTER
        struct C1VEC {
            static constexpr address_t address = 0x018;

            explicit C1VEC(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Interrupt Code, see `InterruptCode`
                    // @note If multiple interrupts are pending, the interrupt with the highest number will be indicated.
                    uint32_t ICODE      : 7;
                    uint32_t padding0   : 1;
                    // Filter Hit Number bits
                    // `11111` = Filter 31
                    // `11110` = Filter 30
                    // ...
                    // `00001` = Filter 1
                    // `00000` = Filter 0
                    uint32_t FILHIT     : 5;
                    uint32_t padding1   : 3;
                    // Transmit Interrupt Flag Code bits
                    // `1000001`-`1111111` = Reserved
                    // `1000000` = No interrupt
                    // `0100000`-`0111111` = Reserved
                    // `0011111` = FIFO 31 Interrupt (TFIF[31] set)
                    // ...
                    // `0000001` = FIFO 1 Interrupt (TFIF[1] set
                    uint32_t TXCODE     : 7;
                    uint32_t padding2   : 1;
                    // Receive Interrupt Flag Code bits
                    // `1000001`-`1111111` = Reserved
                    // `1000000` = No interrupt
                    // `0100000`-`0111111` = Reserved
                    // `0011111` = FIFO 31 Interrupt (RFIF[31] set)
                    // ...
                    // `0000010` = FIFO 2 Interrupt (RFIF[2] set)
                    // `0000001` = FIFO 1 Interrupt (RFIF[1] set)
                    // `0000000` = Reserved. FIFO 0 cannot receive.
                    uint32_t RXCODE     : 7;
                    uint32_t padding3   : 1;
                };
                std::bitset<32> bits;
            };
        };

        // INTERRUPT REGISTER
        struct C1INT {
            static constexpr address_t address = 0x01c;

            explicit C1INT(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit FIFO Interrupt Flag bit
                    uint32_t TXIF       : 1;
                    //  Receive FIFO Interrupt Flag bit
                    uint32_t RXIF       : 1;
                    // Time Base Counter Overflow Interrupt Flag bit
                    uint32_t TBCIF      : 1;
                    // Operation Mode Change Interrupt Flag bit
                    uint32_t MODIF      : 1;
                    // Transmit Event FIFO Interrupt Flag bit
                    uint32_t TEFIF      : 1;
                    uint32_t padding0   : 3;
                    // ECC Error Interrupt Flag bit
                    uint32_t ECCIF      : 1;
                    // SPI CRC Error Interrupt Flag bit
                    uint32_t SPICRCIF   : 1;
                    // Transmit Attempt Interrupt Flag bit
                    uint32_t TXATIF     : 1;
                    // Receive Object Overflow Interrupt Flag bit
                    uint32_t RXOCIF     : 1;
                    // System Error Interrupt Flag bit
                    uint32_t SERRIF     : 1;
                    // CAN Bus Error Interrupt Flag bit
                    uint32_t CERRIF     : 1;
                    // Bus Wake Up Interrupt Flag bit
                    uint32_t WAKIF      : 1;
                    // Invalid Message Interrupt Flag bit
                    uint32_t IVMIF      : 1;
                    // Transmit FIFO Interrupt Enable bit
                    uint32_t TXIE       : 1;
                    // Receive FIFO Interrupt Enable bit
                    uint32_t RXIE       : 1;
                    // Time Base Counter Interrupt Enable bit
                    uint32_t TBCIE      : 1;
                    // Mode Change Interrupt Enable bit
                    uint32_t MODIE      : 1;
                    // Transmit Event FIFO Interrupt Enable bit
                    uint32_t TEFIE      : 1;
                    uint32_t padding1   : 3;
                    // ECC Error Interrupt Enable bit
                    uint32_t ECCIE      : 1;
                    // SPI CRC Error Interrupt Enable bit
                    uint32_t SPICRCIE   : 1;
                    // Transmit Attempt Interrupt Enable bit
                    uint32_t TXATIE     : 1;
                    // Receive FIFO Overflow Interrupt Enable bit
                    uint32_t RXOCIE     : 1;
                    // System Error Interrupt Enable bit
                    uint32_t SERRIE     : 1;
                    // CAN Bus Error Interrupt Enable bit
                    uint32_t CERRIE     : 1;
                    // Bus Wake Up Interrupt Enable bit
                    uint32_t WAKIE      : 1;
                    // Invalid Message Interrupt Enable bit
                    uint32_t IVMIE      : 1;
                };
                std::bitset<32> bits;
            };
        };
        static_assert(sizeof(C1INT) == sizeof(uint32_t));

        // RECEIVE INTERRUPT STATUS REGISTER
        struct C1RXIF {
            static constexpr address_t address = 0x020;

            explicit C1RXIF(uint32_t bits) : bits(bits) {}
            bool IsInterruptPending(uint8_t fifo_index)
            {
                return static_cast<bool>(bits[fifo_index + 1]);
            }
            union {
                struct {
                    uint32_t padding0   : 1;
                    // Receive FIFO Interrupt Pending bits
                    // `1` = One or more enabled receive FIFO interrupts are pending
                    // `0` = No enabled receive FIFO interrupts are pending
                    uint32_t RFIF       : 31;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT INTERRUPT STATUS REGISTER
        struct C1TXIF {
            static constexpr address_t address = 0x024;

            explicit C1TXIF(uint32_t bits) : bits(bits) {}
            bool IsInterruptPending(uint8_t fifo_index)
            {
                return bits[fifo_index + 1];
            }
            bool IsInterruptPendingTXQ()
            {
                return bits[0];
            }
            union {
                struct {
                    // Transmit FIFO/TXQ Interrupt Pending bits
                    // `1` = One or more enabled transmit FIFO/TXQ interrupts are pending
                    // `0` = No enabled transmit FIFO/TXQ interrupt are pending
                    uint32_t TFIF;
                };
                std::bitset<32> bits;
            };
        };

        // RECEIVE OVERFLOW INTERRUPT STATUS REGISTER
        struct C1RXOVIF {
            static constexpr address_t address = 0x028;

            explicit C1RXOVIF(uint32_t bits) : bits(bits) {}
            bool IsInterruptPending(uint8_t fifo_index)
            {
                return bits[fifo_index + 1];
            }
            union {
                struct {
                    uint32_t padding0   : 1;
                    // Receive FIFO Overflow Interrupt Pending bits
                    // 1 = Interrupt is pending
                    // 0 = Interrupt not pending
                    uint32_t RFOVIF     : 31;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT ATTEMPT INTERRUPT STATUS REGISTER
        struct C1TXATIF {
            static constexpr address_t address = 0x02c;

            explicit C1TXATIF(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit FIFO/TXQ Attempt Interrupt Pending bits
                    // `1` = Interrupt is pending
                    // `0` = Interrupt not pending
                    uint32_t TFATIF;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT REQUEST REGISTER
        struct C1TXREQ {
            static constexpr address_t address = 0x030;

            explicit C1TXREQ(uint32_t bits) : bits(bits) {}
            void SetRequestToSend(uint8_t fifo_index)
            {
                bits[fifo_index + 1] = 1;
            }
            void SetTXQRequestToSend()
            {
                bits[0] = 1;
            }
            union {
                struct {
                    // Message Send Request bits
                    // `TXEN` = `1` (Object configured as a Transmit Object)
                    // Setting this bit to `1` requests sending a message.
                    // The bit will automatically clear when the message(s) queued in the object is (are) successfully sent.
                    // This bit can NOT be used for aborting a transmission.
                    // `TXEN` = `0` (Object configured as a Receive Object)
                    // This bit has no effect
                    uint32_t TXREQ;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT/RECEIVE ERROR COUNT REGISTER
        struct C1TREC {
            static constexpr address_t address = 0x034;

            explicit C1TREC(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Receive Error Counter bits
                    uint32_t REC        : 8;
                    // Transmit Error Counter bits
                    uint32_t TEC        : 8;
                    // Transmitter or Receiver is in Error Warning State bit
                    uint32_t EWARN      : 1;
                    // Receiver in Error Warning State bit (128 > REC > 95)
                    uint32_t RXWARN     : 1;
                    // Transmitter in Error Warning State bit (128 > TEC > 95)
                    uint32_t TXWARN     : 1;
                    // Receiver in Error Passive State bit (REC > 127)
                    uint32_t RXBP       : 1;
                    // Transmitter in Error Passive State bit (TEC > 127)
                    uint32_t TXBP       : 1;
                    // Transmitter in Bus Off State bit (TEC > 255)
                    // In Configuration mode, TXBO is set, since the module is not on the bus
                    uint32_t TXBO       : 1;
                    uint32_t padding0   : 10;
                };
                std::bitset<32> bits;
            };
        };

        // BUS DIAGNOSTIC REGISTER 0
        struct C1BDIAG0 {
            static constexpr address_t address = 0x038;

            explicit C1BDIAG0(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Nominal Bit Rate Receive Error Counter bits
                    uint32_t NRERRCNT : 8;
                    // Nominal Bit Rate Transmit Error Counter bits
                    uint32_t NTERRCNT : 8;
                    // Data Bit Rate Receive Error Counter bits
                    uint32_t DRERRCNT : 8;
                    // Data Bit Rate Transmit Error Counter bits
                    uint32_t DTERRCNT : 8;
                };
                std::bitset<32> bits;
            };
        };

        // BUS DIAGNOSTICS REGISTER 1
        struct C1BDIAG1 {
            static constexpr address_t address = 0x03c;

            explicit C1BDIAG1(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Error Free Message Counter bits
                    uint32_t EFMSGCNT   : 16;
                    // During the transmission of a message (or acknowledge bit, or active error flag, or overload
                    // flag), the device wanted to send a dominant level (data or identifier bit logical value `0`), but the
                    // monitored bus value was recessive.
                    uint32_t NBIT0ERR   : 1;
                    // During the transmission of a message (with the exception of the arbitration field), the
                    // device wanted to send a recessive level (bit of logical value `1`), but the monitored bus value was
                    // dominant.
                    uint32_t NBIT1ERR   : 1;
                    // Transmitted message was not acknowledged.
                    uint32_t NACKERR    : 1;
                    // A fixed format part of a received frame has the wrong format
                    uint32_t NFORMERR   : 1;
                    // More than 5 equal bits in a sequence have occurred in a part of a received message
                    // where this is not allowed.
                    uint32_t NSTUFERR   : 1;
                    // The CRC check sum of a received message was incorrect. The CRC of an incoming
                    // message does not match with the CRC Getd from the received data.
                    uint32_t NCRCERR    : 1;
                    uint32_t padding0   : 1;
                    // Device went to bus-off (and auto-recovered).
                    uint32_t TXBOERR    : 1;
                    // Same as for nominal bit rate (see above).
                    uint32_t DBIT0ERR   : 1;
                    // Same as for nominal bit rate (see above).
                    uint32_t DBIT1ERR   : 1;
                    uint32_t padding1   : 1;
                    // Same as for nominal bit rate (see above).
                    uint32_t DFORMERR   : 1;
                    // Same as for nominal bit rate (see above).
                    uint32_t DSTUFERR   : 1;
                    // Same as for nominal bit rate (see above).
                    uint32_t DCRCERR    : 1;
                    // ESI flag of a received CAN FD message was set.
                    uint32_t ESI        : 1;
                    // DLC Mismatch bit
                    // During a transmission or reception, the specified DLC is larger than the PLSIZE of the FIFO element
                    uint32_t DLCMM      : 1;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT EVENT FIFO CONTROL REGISTER
        struct C1TEFCON {
            static constexpr address_t address = 0x040;

            explicit C1TEFCON(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit Event FIFO Not Empty Interrupt Enable bit
                    // `1` = Interrupt enabled for FIFO not empty
                    // `0` = Interrupt disabled for FIFO not empty
                    uint32_t TEFNEIE    : 1;
                    // Transmit Event FIFO Half Full Interrupt Enable bit
                    // `1` = Interrupt enabled for FIFO half full
                    // `0` = Interrupt disabled for FIFO half full
                    uint32_t TEFHIE     : 1;
                    // Transmit Event FIFO Full Interrupt Enable bit
                    // `1` = Interrupt enabled for FIFO full
                    // `0` = Interrupt disabled for FIFO full
                    uint32_t TEFFIE     : 1;
                    // Transmit Event FIFO Overflow Interrupt Enable bit
                    // `1` = Interrupt enabled for overflow event
                    // `0` = Interrupt disabled for overflow event
                    uint32_t TEFOVIE    : 1;
                    uint32_t padding0   : 1;
                    // Transmit Event FIFO Time Stamp Enable bit
                    // `1` = Time Stamp objects in TEF
                    // `0` = Do not Time Stamp objects in TEF
                    uint32_t TEFTSEN    : 1;
                    uint32_t padding1   : 2;
                    // Increment Tail bit
                    // When this bit is set, the FIFO tail will increment by a single message.
                    uint32_t UINC       : 1;
                    uint32_t padding2   : 1;
                    // FIFO Reset bit
                    // `1` = FIFO will be reset when bit is set, cleared by hardware when FIFO was reset. The user should
                    // wait for this bit to clear before taking any action.
                    // `0` = No effect
                    uint32_t FRESET     : 1;
                    uint32_t padding3   : 13;
                    // FIFO Size bits
                    // `0_0000` = FIFO is 1 Message deep
                    // `0_0001` = FIFO is 2 Messages deep
                    // `0_0010` = FIFO is 3 Messages deep
                    // ...
                    // `1_1111` = FIFO is 32 Messages deep
                    uint32_t FSIZE      : 5;
                    uint32_t padding4   : 3;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT EVENT FIFO STATUS REGISTER
        struct C1TEFSTA {
            static constexpr address_t address = 0x044;

            explicit C1TEFSTA(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit Event FIFO Not Empty Interrupt Flag bit
                    // `1` = FIFO is not empty, contains at least one message
                    // `0` = FIFO is empty
                    uint32_t TEFNEIF    : 1;
                    // Transmit Event FIFO Half Full Interrupt Flag bit
                    // `1` = FIFO is ≥ half full
                    // `0` = FIFO is < half full
                    uint32_t TEFHIF     : 1;
                    // Transmit Event FIFO Full Interrupt Flag bit
                    // `1` = FIFO is full
                    // `0` = FIFO is not full
                    uint32_t TEFFIF     : 1;
                    // Transmit Event FIFO Overflow Interrupt Flag bit
                    // `1` = Overflow event has occurred
                    // `0` = No overflow event occurred
                    uint32_t TEFOVIF    : 1;
                    uint32_t padding0   : 28;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT EVENT FIFO USER ADDRESS REGISTER
        struct C1TEFUA {
            static constexpr address_t address = 0x048;

            explicit C1TEFUA(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit Event FIFO User Address bits
                    // A read of this register will return the address where the next object is to be read (FIFO tail).
                    // @note This register is not guaranteed to read correctly in Configuration mode and
                    // should only be accessed when the module is not in Configuration mode.
                    uint32_t TEFUA;
                };
                std::bitset<32> bits;
            };
        };

        // Address 0x04c is reserved

        // TRANSMIT QUEUE CONTROL REGISTER
        struct C1TXQCON {
            static constexpr address_t address = 0x050;

            explicit C1TXQCON(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit Queue Not Full Interrupt Enable bit
                    // `1` = Interrupt enabled for TXQ not full
                    // `0` = Interrupt disabled for TXQ not full
                    uint32_t TXQNIE     : 1;
                    uint32_t padding0   : 1;
                    // Transmit Queue Empty Interrupt Enable bit
                    // `1` = Interrupt enabled for TXQ empty
                    // `0` = Interrupt disabled for TXQ empty
                    uint32_t TXQEIE     : 1;
                    uint32_t padding1   : 1;
                    // Transmit Attempts Exhausted Interrupt Enable bit
                    // `1` = Enable interrupt
                    // `0` = Disable interrupt
                    uint32_t TXATIE     : 1;
                    uint32_t padding2   : 2;
                    // TX Enable
                    // `1` = Transmit Message Queue. This bit always reads as `1`
                    uint32_t TXEN       : 1;
                    // Increment Head bit
                    // When this bit is set, the FIFO head will increment by a single message
                    uint32_t UINC       : 1;
                    // Message Send Request bit
                    // `1` = Requests sending a message; the bit will automatically clear when all the messages queued in
                    // the TXQ are successfully sent.
                    // `0` = Clearing the bit to `0` while set (`1`) will request a message abort.
                    uint32_t TXREQ      : 1;
                    //  FIFO Reset bit
                    // `1` = FIFO will be reset when bit is set; cleared by hardware when FIFO was reset.
                    // User should wait until this bit is clear before taking any action.
                    // `0` = No effect
                    uint32_t FRESET     : 1;
                    uint32_t padding3   : 5;
                    // Message Transmit Priority bits
                    // `00000` = Lowest Message Priority
                    // ...
                    // `11111` = Highest Message Priority
                    uint32_t TXPRI      : 5;
                    // Retransmission Attempts bits
                    // This feature is enabled when CiCON.RTXAT is set.
                    // `00` = Disable retransmission attempts
                    // `01` = Three retransmission attempts
                    // `10` = Unlimited number of retransmission attempts
                    // `11` = Unlimited number of retransmission attempts
                    uint32_t TXAT       : 2;
                    uint32_t padding4   : 1;
                    // FIFO Size bits
                    // `0_0000` = FIFO is 1 Message deep
                    // `0_0001` = FIFO is 2 Messages deep
                    // `0_0010` = FIFO is 3 Messages deep
                    // ...
                    // `1_1111` = FIFO is 32 Messages deep
                    uint32_t FSIZE      : 5;
                    // Payload size, see `cPayloadSizes`
                    uint32_t PLSIZE     : 3;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT QUEUE STATUS REGISTER
        struct C1TXQSTA {
            static constexpr address_t address = 0x054;

            explicit C1TXQSTA(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // Transmit Queue Not Full Interrupt Flag bit
                    uint32_t TXQNIF     : 1;
                    uint32_t padding0   : 1;
                    // Transmit Queue Empty Interrupt Flag bit
                    uint32_t TXQEIF     : 1;
                    uint32_t padding1   : 1;
                    // Transmit Attempts Exhausted Interrupt Pending bit
                    uint32_t TXATIF     : 1;
                    // Error Detected During Transmission bit
                    // `1` = A bus error occurred while the message was being sent
                    // `0` = A bus error did not occur while the message was being sent
                    uint32_t TXERR      : 1;
                    // Message Lost Arbitration Status bit
                    // `1` = Message lost arbitration while being sent
                    // `0` = Message did not lose arbitration while being sent
                    uint32_t TXLARB     : 1;
                    // Message Aborted Status bit
                    // `1` = Message was aborted
                    // `0` = Message completed successfully
                    uint32_t TXABT      : 1;
                    // Transmit Queue Message Index bits
                    // A read of this register will return an index to the message that the
                    // FIFO will next attempt to transmit
                    uint32_t TXQCI      : 5;
                    uint32_t padding2   : 19;
                };
                std::bitset<32> bits;
            };
        };

        // TRANSMIT QUEUE USER ADDRESS REGISTER
        struct C1TXQUA {
            static constexpr address_t address = 0x054;

            explicit C1TXQUA(uint32_t bits) : bits(bits) {}
            union {
                struct {
                    // TXQ User Address bits
                    //
                    // A read of this register will return the address where the next message is to be written (TXQ head)
                    // @note This register is not guaranteed to read correctly in Configuration mode
                    // and should only be accessed when the module is not in Configuration mode.
                    uint32_t TXQUA;
                };
                std::bitset<32> bits;
            };
        };

        // FIFO CONTROL REGISTER
        struct C1FIFOCON {
        private:
            static constexpr address_t base_address = 0x05C;
        public:
            const uint8_t fifo_index;
            const address_t address;

            constexpr C1FIFOCON(uint8_t fifo_index)
                : fifo_index(fifo_index)
                , address(base_address + (3 * fifo_index * sizeof(uint32_t)))
                , TFNRFIE(0)
                , TFHRFHIE(0)
                , TFERFFIE(0)
                , RXOVIE(0)
                 {}

            ~C1FIFOCON() = default;

            union {
                struct {
                    // Transmit/Receive FIFO Not Full/Not Empty Interrupt Enable bit
                    // `TXEN` = `1` (FIFO configured as a Transmit FIFO)
                    // Transmit FIFO Not Full Interrupt Enable
                    // `1` = Interrupt enabled for FIFO not full
                    // `0` = Interrupt disabled for FIFO not full
                    // `TXEN` = `0` (FIFO configured as a Receive FIFO)
                    // Receive FIFO Not Empty Interrupt Enable
                    // `1` = Interrupt enabled for FIFO not empty
                    // `0` = Interrupt disabled for FIFO not empty
                    uint32_t TFNRFIE    : 1;
                    // Transmit/Receive FIFO Half Empty/Half Full Interrupt Enable bit
                    // `TXEN` = `1` (FIFO configured as a Transmit FIFO)
                    // Transmit FIFO Half Empty Interrupt Enable
                    // `1` = Interrupt enabled for FIFO half empty
                    // `0` = Interrupt disabled for FIFO half empty
                    // `TXEN` = `0` (FIFO configured as a Receive FIFO)
                    // Receive FIFO Half Full Interrupt Enable
                    // `1` = Interrupt enabled for FIFO half full
                    // `0` = Interrupt disabled for FIFO half full
                    uint32_t TFHRFHIE   : 1;
                    // Transmit/Receive FIFO Empty/Full Interrupt Enable bit
                    // `TXEN` = `1` (FIFO configured as a Transmit FIFO)
                    // Transmit FIFO Empty Interrupt Enable
                    // `1` = Interrupt enabled for FIFO empty
                    // `0` = Interrupt disabled for FIFO empty
                    // `TXEN` = `0` (FIFO configured as a Receive FIFO)
                    // Receive FIFO Full Interrupt Enable
                    // `1` = Interrupt enabled for FIFO full
                    // `0` = Interrupt disabled for FIFO full
                    uint32_t TFERFFIE   : 1;
                    // Overflow Interrupt Enable bit
                    // `1` = Interrupt enabled for overflow event
                    // `0` = Interrupt disabled for overflow event
                    uint32_t RXOVIE     : 1;
                    // Transmit Attempts Exhausted Interrupt Enable bit
                    // `1` = Enable interrupt
                    // `0` = Disable interrupt
                    uint32_t TXATIE     : 1;
                    // Received Message Time Stamp Enable bit
                    // `1` = Capture time stamp in received message object in RAM.
                    // `0` = Do not capture time stamp
                    uint32_t RXTSEN     : 1;
                    // Auto RTR Enable bit
                    // `1` = When a remote transmit is received, TXREQ will be set.
                    // `0` = When a remote transmit is received, TXREQ will be unaffected
                    uint32_t RTREN      : 1;
                    // TX/RX FIFO Selection bit
                    // `1` = Transmit FIFO
                    // `0` = Receive FIFO
                    uint32_t TXEN       : 1;
                    // Increment Head/Tail bit
                    // `TXEN` = `1` (FIFO configured as a Transmit FIFO)
                    // When this bit is set, the FIFO head will increment by a single message.
                    // `TXEN` = `0` (FIFO configured as a Receive FIFO)
                    // When this bit is set, the FIFO tail will increment by a single message
                    uint32_t UINC       : 1;
                    // Message Send Request bit
                    // `TXEN` = `1` (FIFO configured as a Transmit FIFO)
                    // `1` = Requests sending a message; the bit will automatically clear when all the messages queued in
                    // the FIFO are successfully sent.
                    // `0` = Clearing the bit to `0` while set (`1`) will request a message abort.
                    // `TXEN` = `0` (FIFO configured as a Receive FIFO)
                    // This bit has no effect
                    uint32_t TXREQ      : 1;
                    // FIFO Reset bit
                    uint32_t FRESET     : 1;
                    uint32_t padding0   : 5;
                    // Message Transmit Priority bits, See `C1TXQCON`
                    uint32_t TXPRI      : 5;
                    // Retransmission Attempts bits, See `C1TXQCON`
                    uint32_t TXAT       : 2;
                    uint32_t padding1   : 1;
                    // FIFO Size bits, See `C1TXQCON`
                    uint32_t FSIZE      : 5;
                    // Payload size, See `C1TXQCON`
                    uint32_t PLSIZE     : 3;
                };
                std::bitset<32> bits;
            };
        };

        static consteval std::array<C1FIFOCON, cNumFIFOs> getFifoConfigs() {
            auto configs = create_array<C1FIFOCON, cNumFIFOs>();
            for (size_t i = 0; i < configs.size(); ++i) {
                bool tx = i < cNumTxFIFOs;
                int payload_size = tx ? cTxFIFOPayloadSizes[i] : cRxFIFOPayloadSizes[i - cNumTxFIFOs];
                int fifo_size = tx ? cTxFIFOSizes[i] : cRxFIFOSizes[i - cNumTxFIFOs];
                auto it = std::find(cPayloadSizes.begin(), cPayloadSizes.end(), payload_size);
                if constexpr (!cEnableCANFD) {
                    MCP2518_ASSERT(payload_size == 8 && "Enable CAN FD for extended frame support");
                }
                if (it == cPayloadSizes.end()) {
                    MCP2518_ASSERT(0 && "Payload size not supported");
                }
                configs[i].PLSIZE = it - cPayloadSizes.begin();
                if (fifo_size > cMaxFifoSize) {
                    MCP2518_ASSERT(0 && "Max FIFO size exceeded");
                }
                // Need to explicity initialize each field to please the compiler
                configs[i].FSIZE    = fifo_size - 1;
                configs[i].padding1 = 0;
                configs[i].TXAT     = 0;
                configs[i].TXPRI    = 0;
                configs[i].padding0 = 0;
                configs[i].FRESET   = 1;
                configs[i].TXREQ    = 0;
                configs[i].UINC     = 0;
                configs[i].TXEN     = tx ? 1 : 0;
                configs[i].RTREN    = 0;
                configs[i].RXTSEN   = 1;
                configs[i].TXATIE   = 0;
                configs[i].RXOVIE   = 0;
                configs[i].TFERFFIE = 0;
                configs[i].TFHRFHIE = 0;
                configs[i].TFNRFIE  = 1;
            }
            return configs;
        }

        // FIFO STATUS REGISTER
        struct C1FIFOSTA {
        private:
            static constexpr address_t base_address = 0x060;
        public:
            const uint8_t fifo_index;
            const address_t address;

            constexpr explicit C1FIFOSTA(uint8_t fifo_index, uint32_t bits)
                : fifo_index(fifo_index)
                , address(base_address + (3 * fifo_index * sizeof(uint32_t)))
                , bits(bits) {}
            union {
                struct {
                    // Transmit/Receive FIFO Not Full/Not Empty Interrupt Flag bit
                    // TXEN = 1 (FIFO is configured as a Transmit FIFO)
                    // Transmit FIFO Not Full Interrupt Flag
                    // `1` = FIFO is not full
                    // `0` = FIFO is full
                    // TXEN = 0 (FIFO is configured as a Receive FIFO)
                    // Receive FIFO Not Empty Interrupt Flag
                    // `1` = FIFO is not empty, contains at least one message
                    // `0` = FIFO is empty
                    uint32_t TFNRFIF    : 1;
                    // Transmit/Receive FIFO Half Empty/Half Full Interrupt Flag bit
                    // TXEN = 1 (FIFO is configured as a Transmit FIFO)
                    // Transmit FIFO Half Empty Interrupt Flag
                    // `1` = FIFO is ≤ half full
                    // `0` = FIFO is > half full
                    // TXEN = 0 (FIFO is configured as a Receive FIFO)
                    // Receive FIFO Half Full Interrupt Flag
                    // `1` = FIFO is ≤ half full
                    // `0` = FIFO is < half full
                    uint32_t TFHRFHIF   : 1;
                    // Transmit/Receive FIFO Empty/Full Interrupt Flag bit
                    // TXEN = 1 (FIFO is configured as a Transmit FIFO)
                    // Transmit FIFO Empty Interrupt Flag
                    // `1` = FIFO is empty
                    // `0` = FIFO is not empty; at least one message queued to be transmitted
                    // TXEN = 0 (FIFO is configured as a Receive FIFO)
                    // Receive FIFO Full Interrupt Flag
                    // `1` = FIFO is full
                    // `0` = FIFO is not full
                    uint32_t TFERFFIF   : 1;
                    // Receive FIFO Overflow Interrupt Flag bit
                    // TXEN = 1 (FIFO is configured as a Transmit FIFO)
                    // Unused, Read as ‘0’
                    // TXEN = 0 (FIFO is configured as a Receive FIFO)
                    // `1` = Overflow event has occurred
                    // `0` = No overflow event has occurred
                    uint32_t RXOVIF     : 1;
                    // Transmit Attempts Exhausted Interrupt Pending bit
                    // TXEN = 1 (FIFO is configured as a Transmit FIFO)
                    // `1` = Interrupt pending
                    // `0` = Interrupt not pending
                    // TXEN = 0 (FIFO is configured as a Receive FIFO)
                    // Read as `0`
                    uint32_t TXATIF     : 1;
                    // Error Detected During Transmission bit(2)(3)
                    // `1` = A bus error occurred while the message was being sent
                    // `0` = A bus error did not occur while the message was being sent
                    uint32_t TXERR      : 1;
                    // Message Lost Arbitration Status bit(2)(3)
                    // `1` = Message lost arbitration while being sent
                    // `0` = Message did not lose arbitration while being sent
                    uint32_t TXLARB     : 1;
                    // Message Aborted Status bit
                    // `1` = Message was aborted
                    // `0` = Message completed successfully
                    uint32_t TXABT      : 1;
                    // FIFO Message Index bits
                    // `TXEN` = `1` (FIFO is configured as a Transmit FIFO)
                    // A read of this bit field will return an index to the message that the FIFO will next attempt to transmit.
                    // `TXEN` = `0` (FIFO is configured as a Receive FIFO)
                    // A read of this bit field will return an index to the message that the FIFO will use to save the next
                    // message
                    uint32_t FIFOCI     : 5;
                    uint32_t padding0   : 19;
                };
                std::bitset<32> bits;
            };
        };

        // FIFO USER ADDRESS REGISTER
        struct C1FIFOUA {
        private:
            static constexpr address_t base_address = 0x064;
        public:
            const uint8_t fifo_index;
            const address_t address;

            constexpr explicit C1FIFOUA(uint8_t fifo_index, uint32_t bits) 
                : fifo_index(fifo_index)
                , address(base_address + (3 * fifo_index * sizeof(uint32_t)))
                , bits(bits) {}
            union {
                struct {
                    // FIFO User Address bits
                    // TXEN = `1` (FIFO is configured as a Transmit FIFO)
                    // A read of this register will return the address where the next message is to be written (FIFO head).
                    // TXEN = `0` (FIFO is configured as a Receive FIFO)
                    // A read of this register will return the address where the next message is to be read (FIFO tail).
                    uint32_t FIFOUA;
                };
                std::bitset<32> bits;
            };
        };

        // FILTER CONTROL REGISTER
        struct C1FLTCON {
        private:
            static constexpr address_t base_address = 0x01d0;
        public:
            const uint8_t filter_group_index;
            const address_t address;

            explicit C1FLTCON(uint8_t filter_group_index, uint32_t bits)
                : filter_group_index(filter_group_index)
                , address(base_address + (filter_group_index * sizeof(uint32_t)))
                , bits(bits) {}

            union {
                struct {
                    // Pointer to FIFO when Filter x hits bits
                    // `1_1111` = Message matching filter is stored in FIFO 31
                    // `1_1110` = Message matching filter is stored in FIFO 30
                    // ........
                    // `0_0010` = Message matching filter is stored in FIFO 2
                    // `0_0001` = Message matching filter is stored in FIFO 1
                    // `0_0000` = Reserved FIFO 0 is the TX Queue and cannot receive messages
                    uint8_t FBP : 5;
                    uint8_t padding : 2;
                    // FLTENx: Enable Filter x to Accept Messages bit
                    // `1` = Filter is enabled
                    // `0` = Filter is disabled
                    uint8_t FLTEN : 1;
                } FLTCON[4];
                std::bitset<32> bits;
            };
        };

        // FILTER OBJECT REGISTER
        struct C1FLTOBJ {
        private:
            static constexpr address_t base_address = 0x1f0;
        public:
            const uint8_t filter_index;
            const address_t address;

            constexpr explicit C1FLTOBJ(uint8_t filter_index, uint32_t bits)
                : filter_index(filter_index)
                , address(base_address + (2 * filter_index * sizeof(uint32_t)))
                , bits(bits) {}

            union {
                struct {
                    // Standard Identifier filter bits
                    uint32_t SID        : 11;
                    // Extended Identifier filter bits
                    uint32_t EID        : 18;
                    // Standard Identifier filter bit
                    uint32_t SID11      : 1;
                    // Extended Identifier Enable bit
                    // If `MIDE` = `1`:
                    // `1` = Match only messages with extended identifier
                    // `0` = Match only messages with standard identifier
                    uint32_t EXIDE      : 1;
                    uint32_t padding0   : 1;
                };
                std::bitset<32> bits;
            };
        };

        // MASK REGISTER
        struct C1MASK {
        private:
            static constexpr address_t base_address = 0x1f4;
        public:
            const uint8_t filter_index;
            const address_t address;

            explicit C1MASK(uint8_t filter_index, uint32_t bits)
            : filter_index(filter_index)
            , address(base_address + (2 * filter_index * sizeof(uint32_t)))
            , bits(bits) {}

            union {
                struct {
                    // Standard Identifier Mask bits
                    uint32_t MSID        : 11;
                    // Extended Identifier Mask bits
                    uint32_t MEID        : 18;
                    // Standard Identifier Mask bit
                    uint32_t MSID11      : 1;
                    // Identifier Receive mode bit
                    // `1` = Match only message types (standard or extended ID) that correspond to EXIDE bit in filter
                    // `0` = Match both standard and extended message frames if filters match
                    uint32_t MEIDE       : 1;
                    uint32_t padding0    : 1;
                };
                std::bitset<32> bits;
            };
        };
    };

    template<size_t payload_size = 8>
    struct TXMessage {
        struct Header {
            // Standard Identifier
            uint32_t SID : 11;
            // Extended Identifier
            uint32_t EID : 18;
            //  In FD mode the standard ID can be extended to 12 bit using r1
            uint32_t SID11 : 1;
            uint32_t padding0 : 2;
            // Data Length Code
            uint32_t DLC : 4;
            // Identifier Extension Flag; distinguishes between base and extended format
            uint32_t IDE : 1;
            //  Remote Transmission Request; not used in CAN FD
            uint32_t RTR : 1;
            // Bit Rate Switch; selects if data bit rate is switched
            uint32_t BRS : 1;
            // FD Frame; distinguishes between CAN and CAN FD formats
            uint32_t FDF : 1;
            // Error Status Indicator
            // In CAN to CAN gateway mode (CiCON.ESIGM=1), the transmitted ESI flag is a “logical OR” of T1.ESI
            // and error passive state of the CAN FD Controller;
            // In normal mode ESI indicates the error status
            // `1` = Transmitting node is error passive
            // `0` = Transmitting node is error active
            uint32_t ESI : 1;
            // Sequence to keep track of transmitted messages in Transmit Event FIFO
            uint32_t SEQ : 23;
        } header;
        std::array<uint8_t, payload_size> data;
    };

    template<size_t payload_size = 8>
    struct RXMessage {
        struct Header {
            // Standard Identifier
            uint32_t SID : 11;
            // Extended Identifier
            uint32_t EID : 18;
            //  In FD mode the standard ID can be extended to 12 bit using r1
            uint32_t SID11    : 1;
            uint32_t padding0 : 2;
            // Data Length Code
            uint32_t DLC : 4;
            // Identifier Extension Flag; distinguishes between base and extended format
            uint32_t IDE : 1;
            //  Remote Transmission Request; not used in CAN FD
            uint32_t RTR : 1;
            // Bit Rate Switch; selects if data bit rate is switched
            uint32_t BRS : 1;
            // FD Frame; distinguishes between CAN and CAN FD formats
            uint32_t FDF : 1;
            // Error Status Indicator
            // `1` = Transmitting node is error passive
            // `0` = Transmitting node is error active
            uint32_t ESI      : 1;
            uint32_t padding1 : 2;
            // Filter Hit, number of filter that matched
            uint32_t FILTHIT  : 5;
            uint32_t padding2 : 16;
            // Receive Message Time Stamp
            uint32_t RXMSGTS;
        } header;
        std::array<uint8_t, payload_size> data;
    };

    struct TXEvent {
        struct Header {
            // Standard Identifier
            uint32_t SID : 11;
            // Extended Identifier
            uint32_t EID : 18;
            //  In FD mode the standard ID can be extended to 12 bit using r1
            uint32_t SID11 : 1;
            uint32_t padding0 : 2;
            // Data Length Code
            uint32_t DLC : 4;
            // Identifier Extension Flag; distinguishes between base and extended format
            uint32_t IDE : 1;
            //  Remote Transmission Request; not used in CAN FD
            uint32_t RTR : 1;
            // Bit Rate Switch; selects if data bit rate is switched
            uint32_t BRS : 1;
            // FD Frame; distinguishes between CAN and CAN FD formats
            uint32_t FDF : 1;
            // Error Status Indicator
            // In CAN to CAN gateway mode (CiCON.ESIGM=1), the transmitted ESI flag is a “logical OR” of T1.ESI
            // and error passive state of the CAN FD Controller;
            // In normal mode ESI indicates the error status
            // `1` = Transmitting node is error passive
            // `0` = Transmitting node is error active
            uint32_t ESI : 1;
            // Sequence to keep track of transmitted messages in Transmit Event FIFO
            uint32_t SEQ : 23;
            // Transmit Message Time Stamp
            uint32_t TXMSGTS;
        } header;
    };

    /***********************************
     *  Constant Functions and Fields  *
     ***********************************/

    // Memory map
    static constexpr address_t cMessageRamBaseAddress = 0x400;
    static constexpr address_t cRamSize = 0x800;
    static consteval address_t GetTEFBaseAddress(address_t base_address) { return base_address; }

    static consteval address_t GetTXQBaseAddress(address_t tef_base_address)
    {
        return tef_base_address + cTEFSize * sizeof(TXEvent);
    }

    static consteval std::array<address_t, cNumTxFIFOs> GetTxFIFOsBaseAddresses(
        address_t txq_base_address)
    {
        std::array<address_t, cNumTxFIFOs> addresses;
        addresses[0] = txq_base_address + cTXQSize * sizeof(TXMessage<cTXQPayloadSize>);
        for (size_t i = 1; i < addresses.size(); ++i) {
            addresses[i] = addresses[i - 1]
                + (cTxFIFOSizes[i - 1]
                    * (sizeof(TXMessage<>::Header) + cTxFIFOPayloadSizes[i - 1]));
        }
        return addresses;
    }

    static consteval std::array<address_t, cNumRxFIFOs> GetRxFIFOsBaseAddresses(
        address_t last_tx_fifo_base_address)
    {
        std::array<address_t, cNumRxFIFOs> addresses;
        addresses[0] = last_tx_fifo_base_address
            + (cTxFIFOSizes.back() * sizeof(TXMessage<cTxFIFOPayloadSizes.back()>));
        for (size_t i = 1; i < addresses.size(); ++i) {
            addresses[i] = addresses[i - 1]
                + (cRxFIFOSizes[i - 1]
                    * (sizeof(RXMessage<>::Header) + cRxFIFOPayloadSizes[i - 1]));
        }
        if (addresses.back() + cRxFIFOSizes.back() * sizeof(RXMessage<cRxFIFOPayloadSizes.back()>)
            > cMessageRamBaseAddress + cRamSize) {
            MCP2518_ASSERT(0 && "FIFO configuration does not fit in RAM");
        }
        return addresses;
    }

    static constexpr std::array<size_t, 8> cPayloadSizes = { 8, 12, 16, 20, 24, 32, 48, 64 };
    static constexpr size_t cMaxFifoSize = 32;
    static constexpr uint8_t cNumCommandBits = 4;
    static constexpr uint8_t cNumAddressBits = 12;
    static constexpr int cSpiClockHz = 16 * 1000 * 1000; // 16 MHz
    static constexpr int cSysClockHz = cOscillatorFreqHz / cSysClockDivider;
    static_assert(cSpiClockHz <= ((cSysClockHz / 2) * 85) / 100,
        "SPI clock should be no greater than 85% of SYS_CLK / 2");
    static constexpr const char* TAG = "MCP2518";

    static constexpr size_t cMaxTxPayloadSize = std::max(
        sizeof(TXMessage<cTXQPayloadSize>),
        sizeof(TXMessage<*std::max_element(cTxFIFOPayloadSizes.begin(), cTxFIFOPayloadSizes.end())>)
    );

    static constexpr size_t cMaxRxPayloadSize = sizeof(
        RXMessage<*std::max_element(cRxFIFOPayloadSizes.begin(), cRxFIFOPayloadSizes.end())>);

    /*******************
     *  Static fields  *
     *******************/
    inline static bool _isr_service_installed = false;

    /***********************
     *  Private Functions  *
     ***********************/

    /**
     * @brief Set all configuration registers with default and compile-time values
     *
     */
    void configDefault();

    /**
     * @brief Read a single register
     *
     * @param address 12 bits, `address` field of `Registers` structs
     * @return uint32_t raw value, can be used in constructor of `Registers` structs
     *
     * @note This function doesn't accept and return a generic `Register` type to avoid
     *       runtime polymorphism. It doesn't support static polymorphism through templates
     *       to avoid increasing binary size.
     */
    uint32_t readRegister(address_t address) const;

    /**
     * @brief Write a single register
     *
     * @param address 12 bits, `address` field of `Registers` structs
     * @param value raw 32 bit value of register, `bits` field of `Registers` structs
     *
     * @note This function doesn't accept and return a generic `Register` type to avoid
     *       runtime polymorphism. It doesn't support static polymorphism through templates
     *       to avoid increasing binary size.
     */
    void writeRegister(address_t address, uint32_t value) const;

    /**
     * @brief Reads from RX FIFO without checking that a message is available
     * 
     * @param frame Output. CAN 2.0 frame
     * @param timestamp Output. Timestamp in microseconds
     * 
     * @note The timestamp is according to MCP2518's internal clock and is not
     *       related to any timer on the ESP
     */
    void readMessageInternal(uint8_t fifo_index, can_frame_t& frame, uint32_t& timestamp);

    /**
     * @brief Reads from RX FIFO without checking that a message is available
     * 
     * @param frame Output. CAN FD frame
     * @param timestamp Output. Timestamp in microseconds
     * 
     * @note The timestamp is according to MCP2518's internal clock and is not
     *       related to any timer on the ESP
     */
    void readMessageInternal(uint8_t fifo_index, can_fd_frame_t& frame, uint32_t& timestamp);

    /**
     * @brief Get the current RX FIFO append point
     *
     * @param rx_fifo_index RX FIFO index 0-31
     * @return address_t 12 bit address of the next RXMessage in the MCP2518's RAM
     *
     */
    address_t getRxFifoAddress(size_t rx_fifo_index) const;

    /**
     * @brief Increments the RX FIFO tail locally and on MCP2518
     *
     * @param rx_fifo_index FIFO index 0-31
     */
    void incrementRxFifoAddress(size_t rx_fifo_index)
    {
        int config_index = rx_fifo_index + cNumTxFIFOs;
        _fifo_configs[config_index].UINC = 1;
        writeRegister(_fifo_configs[config_index].address, _fifo_configs[config_index].bits.to_ulong());
    }

    /**
     * @brief Get the current TX FIFO append point
     *
     * @param tx_fifo_index TX FIFO index 0-31
     * @return address_t 12 bit address of the next TXMessage in the MCP2518's RAM
     *
     */
    address_t getTxFifoAddress(size_t tx_fifo_index) const;

    /********************
     *  Private fields  *
     ********************/

    const address_t _tef_base_address;
    const address_t _txq_base_address;
    const std::array<address_t, cNumTxFIFOs> _tx_fifos_base_addresses;
    const std::array<address_t, cNumRxFIFOs> _rx_fifos_base_addresses;

    uint8_t _tef_index = 0;
    uint8_t _txq_index = 0;

    std::array<Registers::C1FIFOCON, cNumFIFOs> _fifo_configs;

    bool _tx_fifo_full;
    bool _rx_fifo_not_empty;

    spi_device_handle_t _spi = nullptr;
    gpio_num_t _rx_intr_pin = GPIO_NUM_NC;
    gpio_num_t _tx_intr_pin = GPIO_NUM_NC;
    void* _rx_buffer = nullptr;
    void* _tx_buffer = nullptr;
    rx_callback_t _rxCb = nullptr;
    void* _userData = nullptr;
    tx_callback_t _txCb = nullptr;
    int32_t _nominalBitrate = 0;
    int32_t _dataBitrate = 0;
};

} // namespace cbm
