#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_intr_types.h"

#include "CAN.hpp"
#include "MCP2518.hpp"

constinit cbm::MCP2518 mcp;

constexpr gpio_num_t cPrimarySpiCsPin = GPIO_NUM_15;
constexpr gpio_num_t cSpiMosiPin      = GPIO_NUM_12;
constexpr gpio_num_t cSpiMisoPin      = GPIO_NUM_13;
constexpr gpio_num_t cSpiClkPin       = GPIO_NUM_14;
constexpr gpio_num_t cRxIntrPin       = GPIO_NUM_4;
constexpr gpio_num_t cTxIntrPin       = GPIO_NUM_27;
constexpr spi_host_device_t cSpiHost  = SPI2_HOST;

static const char* TAG = "MCP2518FD Example";

extern "C" void app_main(void)
{
    spi_bus_config_t buscfg = {
        .mosi_io_num = cSpiMosiPin,
        .miso_io_num = cSpiMisoPin,
        .sclk_io_num = cSpiClkPin,
        .data2_io_num = GPIO_NUM_NC,
        .data3_io_num = GPIO_NUM_NC,
        .data4_io_num = GPIO_NUM_NC,
        .data5_io_num = GPIO_NUM_NC,
        .data6_io_num = GPIO_NUM_NC,
        .data7_io_num = GPIO_NUM_NC,
        .data_io_default_level = false,
        .max_transfer_sz = 0,
        .flags = 0,
        .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
        .intr_flags = 0,
    };

    ESP_ERROR_CHECK(spi_bus_initialize(cSpiHost, &buscfg, SPI_DMA_CH_AUTO));

    cbm::MCP2518::Config cfg = {
        .spi_host    = cSpiHost,
        .cs_pin      = cPrimarySpiCsPin,
        .tx_intr_pin = cTxIntrPin,
        .rx_intr_pin = cRxIntrPin,
    };

    mcp.begin(cfg);
    cbm::can_frame_t frame;
    for (;;) {
        uint32_t ts;
        if (mcp.ReadMessage(frame, ts) != cbm::MCP2518::Status::NO_MSG) {
            mcp.SendMessage(frame);
            ESP_LOGI(TAG, "Frame received at %luus", ts);
        }
    }
}
