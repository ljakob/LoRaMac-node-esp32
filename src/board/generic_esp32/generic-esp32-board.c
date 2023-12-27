/* In this implementation, milliseconds units are used throughout. Ticks are milliseconds. */
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"

#include "freertos/task.h"

// from LoRaMac
#include "board.h"
#include "timer.h"

// ANSI
#include <string.h>
#include <sys/time.h>

#define ESP_INTR_FLAG_DEFAULT   0

#define HZ_PER_KHZ  1000
#define KHZ_PER_MHZ 1000
#define HZ_PER_MHZ  (HZ_PER_KHZ * KHZ_PER_MHZ)

#define MS_PER_S    1000
#define US_PER_MS   1000
#define US_PER_S    (MS_PER_S * US_PER_MS)

#define SX126X_MAX_SPI_CLOCK_SPEED_MHZ      16
#define SX126X_NUM_COMMAND_BITS             8
#define SX126X_NUM_REGISTER_ADDRESS_BITS    16
#define SX126X_NUM_COMMAND_ADDRESS_BITS     0
#define SX126X_NUM_BUFFER_OFFSET_BITS       8

typedef struct spi_s {
    spi_device_handle_t handle;
    gpio_num_t miso;
    gpio_num_t mosi;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t reset;
    gpio_num_t irq_dio1;
#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    gpio_num_t busy;
#endif
} spi_c;

static spi_c lora_spi;

static const char *TAG = "ESP32Board";

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

static esp_timer_handle_t irq_timer_handle;
static uint32_t rtc_timer_context;
static uint32_t alarm_start_time;

static void IrqTimerExpiryCallback(void* arg) {
    TimerIrqHandler();
}

void BoardInitMcu( void )
{
}

void BoardInitPeriph(void)
{
    // Create timer
    const esp_timer_create_args_t timer_args = {
        .callback = &IrqTimerExpiryCallback,
        .name = "SX126X IRQ Timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &irq_timer_handle));

    lora_spi.miso = CONFIG_SX126X_SPI_MISO_GPIO;
    lora_spi.mosi = CONFIG_SX126X_SPI_MOSI_GPIO;
    lora_spi.sclk = CONFIG_SX126X_SPI_SCLK_GPIO;
    lora_spi.cs = CONFIG_SX126X_SPI_CS_GPIO;
    lora_spi.reset = CONFIG_SX126X_RESET_GPIO;
    lora_spi.irq_dio1 = CONFIG_SX126X_IRQ_DIO1_GPIO;
#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    lora_spi.busy = CONFIG_SX126X_BUSY_GPIO;

    gpio_config_t busy_conf = {
        (1ULL<<lora_spi.busy),
        GPIO_MODE_INPUT,
        GPIO_PULLUP_DISABLE,
        GPIO_PULLDOWN_ENABLE,
        GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&busy_conf));
#endif

    gpio_config_t cfg_reset_output = {
        (1ULL<<lora_spi.reset),
        GPIO_MODE_OUTPUT,
        GPIO_PULLUP_ENABLE,
        GPIO_PULLDOWN_DISABLE,
        GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&cfg_reset_output));

    CRITICAL_SECTION_BEGIN();

    // SPI initialization
    spi_bus_config_t buscfg = {
        .miso_io_num = lora_spi.miso,
        .mosi_io_num = lora_spi.mosi,
        .sclk_io_num = lora_spi.sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SX126X_MAX_SPI_CLOCK_SPEED_MHZ * HZ_PER_MHZ,
        .mode = 0,
        .spics_io_num = lora_spi.cs,
        .command_bits = SX126X_NUM_COMMAND_BITS,
        .address_bits = SX126X_NUM_REGISTER_ADDRESS_BITS,
        .queue_size = 1 // using ESP32 synchronous SPI API, will only ever be one transaction "in the air" at a time.
        // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/spi_master.html#_CPPv4N29spi_device_interface_config_t10queue_sizeE
    };
    ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)CONFIG_SX126X_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_ERROR_CHECK(spi_bus_add_device((spi_host_device_t)CONFIG_SX126X_SPI_HOST, &devcfg, &lora_spi.handle));

    CRITICAL_SECTION_END();
}

void BoardResetMcu( void )
{
    ESP_LOGI(TAG, "restating from BoardResetMcu");
    esp_restart();
}

void BoardCriticalSectionBegin( uint32_t *mask )
{
    taskENTER_CRITICAL(&my_spinlock);
}

void BoardCriticalSectionEnd( uint32_t *mask )
{
    taskEXIT_CRITICAL(&my_spinlock);
}

void DelayMs( uint32_t ms )
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

#include "esp_efuse.h"
#include "esp_efuse_table.h"

uint32_t BoardGetRandomSeed( void )
{
    uint8_t mac_addr_bin[6];
    ESP_ERROR_CHECK(esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, &mac_addr_bin, sizeof(mac_addr_bin) * 8 /* bits */));
    return (mac_addr_bin[5] << 0) | (mac_addr_bin[4] << 8) | (mac_addr_bin[3] << 16) | (mac_addr_bin[2] << 24);
}

void BoardGetUniqueId( uint8_t *id )
{
    uint8_t mac_addr_bin[6];
    ESP_ERROR_CHECK(esp_efuse_read_field_blob(ESP_EFUSE_MAC_FACTORY, &mac_addr_bin, sizeof(mac_addr_bin) * 8 /* bits */));

    id[7] = mac_addr_bin[5];
    id[6] = mac_addr_bin[4];
    id[5] = mac_addr_bin[3];
    id[4] = mac_addr_bin[2];
    id[3] = mac_addr_bin[1];
    id[2] = mac_addr_bin[0];
    id[1] = mac_addr_bin[5] ^ mac_addr_bin[3];
    id[0] = mac_addr_bin[4] ^ mac_addr_bin[2];
}

// EEPROM

#include "nvs.h"

#include <stdio.h>
#include <strings.h>
#include <inttypes.h>

const static char *namespace = "lrm_ee";

LmnStatus_t EepromMcuReadBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    nvs_handle_t handle;
    int res = nvs_open(namespace, NVS_READONLY, &handle);
    if (res == ESP_ERR_NOT_FOUND)
    {
        ESP_LOGW(TAG, "NVS namespace not found - return zero");
        bzero(buffer, size);
        return LMN_STATUS_OK;
    }
    else if ( res != ESP_OK )
    {
        ESP_LOGW(TAG, "NVS open failed %s", strerror(res));
        return LMN_STATUS_ERROR;
    }

    char name[32];
    sprintf(name, "ee_%" PRIx16, addr);
    ESP_LOGD(TAG, "read eeprom:%s", name);

    LmnStatus_t status = LMN_STATUS_ERROR;

    size_t total;
    res = nvs_get_blob(handle, name, NULL, &total);

    if ( res == ESP_ERR_NOT_FOUND )
    {
        ESP_LOGW(TAG, "key not found - return zero");
        bzero(buffer, size);
        status = LMN_STATUS_OK;
    }
    else if ( res != ESP_OK )
    {
        ESP_LOGI(TAG, "reading1 failed %s", strerror(res));
    }
    else if ( total != size )
    {
        ESP_LOGI(TAG, "size mismatch expected %i found %i", (int)size, (int)total);
    }
    else // OK
    {
        res = nvs_get_blob(handle, name, buffer, &total);
        if ( res == ESP_OK )
        {
            // all fine
            status = LMN_STATUS_OK;
        }
        else
        {
            ESP_LOGI(TAG, "reading2 failed %s", strerror(res));
        }
    }

    nvs_close(handle);

    return status;
}

LmnStatus_t EepromMcuWriteBuffer( uint16_t addr, uint8_t *buffer, uint16_t size )
{
    nvs_handle_t handle;
    nvs_open(namespace, NVS_READONLY, &handle);
    return 42;
}

// RTC

#include "timer.h"


// MCU Wake Up Time
#define MIN_ALARM_DELAY                             3 // in ticks

RTC_DATA_ATTR static uint32_t rtc_value0;
RTC_DATA_ATTR static uint32_t rtc_value1;

void RtcBkupWrite( uint32_t data0, uint32_t data1 )
{
    rtc_value0 = data0;
    rtc_value1 = data1;
}

void RtcBkupRead( uint32_t *data0, uint32_t *data1 )
{
  *data0 = rtc_value0;
  *data1 = rtc_value1;
}

/*!
 * \brief Gets the system time with the number of seconds elapsed since epoch
 *
 * \param [OUT] milliseconds Number of milliseconds elapsed since epoch
 * \retval seconds Number of seconds elapsed since epoch
 */
uint32_t RtcGetCalendarTime( uint16_t *milliseconds )
{
    int64_t time_us = esp_timer_get_time();
    uint32_t time_s = (uint32_t)(time_us / US_PER_S);

    *milliseconds = (uint16_t)((time_us / US_PER_MS) % MS_PER_S);

    return time_s;
}

/*!
 * \brief returns the wake up time in ticks
 *
 * \retval wake up time in ticks
 */
uint32_t RtcGetMinimumTimeout( void )
{
    return( MIN_ALARM_DELAY );
}

/*!
 * \brief Get the RTC timer elapsed time since the last Alarm was set
 *
 * \retval RTC Elapsed time since the last alarm in ticks.
 */
uint32_t RtcGetTimerElapsedTime( void )
{
    int64_t current_time_ms;
    uint32_t elapsed_time_ms;

    current_time_ms = esp_timer_get_time() / US_PER_MS;
    elapsed_time_ms = current_time_ms - alarm_start_time; 

    return elapsed_time_ms;
}

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
uint32_t RtcGetTimerValue( void )
{
    return (esp_timer_get_time() / US_PER_MS);
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    return milliseconds;
}


/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
TimerTime_t RtcTick2Ms( uint32_t tick )
{
    return tick;
}


/*!
 * \brief Sets the RTC timer reference
 *
 * \retval value Timer reference value in ticks
 */
uint32_t RtcSetTimerContext( void )
{
    rtc_timer_context = esp_timer_get_time() / US_PER_MS;

    return rtc_timer_context;
}


/*!
 * \brief Gets the RTC timer reference
 *
 * \retval value Timer reference value in ticks
 */
uint32_t RtcGetTimerContext( void ) {
    return rtc_timer_context;
}


/*!
 * \brief Sets the alarm
 *
 * \note The alarm is set at now (read in this funtion) + timeout
 *
 * \param timeout [IN] Duration of the Timer ticks
 */
void RtcSetAlarm( uint32_t timeout )
{
    uint32_t timeout_us = timeout * US_PER_MS;

    alarm_start_time = esp_timer_get_time() / US_PER_MS;

    if (esp_timer_is_active(irq_timer_handle)) {
        ESP_ERROR_CHECK(esp_timer_restart(irq_timer_handle, timeout_us));
    }
    else {
        ESP_ERROR_CHECK(esp_timer_start_once(irq_timer_handle, timeout_us));
    }
}

/*!
 * \brief Stops the Alarm
 */
void RtcStopAlarm( void )
{
    if (esp_timer_is_active(irq_timer_handle)) {
        ESP_ERROR_CHECK(esp_timer_stop(irq_timer_handle));
    }
}



// RADIO

#include "sx126x-board.h"

#define BOARD_TCXO_WAKEUP_TIME                      0

static RadioOperatingModes_t OperatingMode;

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
void SX126xWaitOnBusy( void )
{
    while(gpio_get_level(lora_spi.busy) == 1) {
        vTaskDelay(pdTICKS_TO_MS(1));
    };
}
#endif

/*!
 * \brief HW Reset of the radio
 */
void SX126xReset( void )
{
    ESP_ERROR_CHECK(gpio_set_level(lora_spi.reset, 0));

    // Hold low for at least 100us.
    // SX1261/2 Datasheet, Rev 1.1 Section 8.1 Reset
    DelayMs(1);

    ESP_ERROR_CHECK(gpio_set_level(lora_spi.reset, 1));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    // Wait for chip to be ready.
    SX126xWaitOnBusy();
#endif

    SX126xSetOperatingMode(MODE_STDBY_RC);
}

/*!
 * \brief Wakes up the radio
 */
void SX126xWakeup( void )
{
    CRITICAL_SECTION_BEGIN( );

    ESP_ERROR_CHECK(gpio_set_level(lora_spi.cs, 0));
    ESP_ERROR_CHECK(gpio_set_level(lora_spi.cs, 1));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    // Wait for chip to be ready.
    SX126xWaitOnBusy();
#endif

    // Update operating mode context variable
    SX126xSetOperatingMode(MODE_STDBY_RC);

    CRITICAL_SECTION_END( );
}

/*!
 * \brief Initializes the RF Switch I/Os pins interface
 */
void SX126xAntSwOn( void )
{
    // No antenna switch available on this board design.
}

/*!
 * \brief De-initializes the RF Switch I/Os pins interface
 *
 * \remark Needed to decrease the power consumption in MCU low power modes
 */
void SX126xAntSwOff( void )
{
    // No antenna switch available on this board design.
}

/*!
 * \brief Gets the Defines the time required for the TCXO to wakeup [ms].
 *
 * \retval time Board TCXO wakeup time in ms.
 */
uint32_t SX126xGetBoardTcxoWakeupTime( void )
{
    return BOARD_TCXO_WAKEUP_TIME;
}

/*!
 * \brief Gets the device ID
 *
 * \retval id Connected device ID
 */
uint8_t SX126xGetDeviceId( void )
{
#ifdef SX1261MBXBAS
    return SX1261;
#elif defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    return SX1262;
#endif
}

/*!
 * \brief Gets the current Radio OperationMode variable
 *
 * \retval      RadioOperatingModes_t last operating mode
 */
RadioOperatingModes_t SX126xGetOperatingMode( void )
{
    return OperatingMode;
}

/*!
 * \brief Sets/Updates the current Radio OperationMode variable.
 *
 * \remark WARNING: This function is only required to reflect the current radio
 *                  operating mode when processing interrupts.
 *
 * \param [in] mode           New operating mode
 */
void SX126xSetOperatingMode( RadioOperatingModes_t mode )
{
    OperatingMode = mode;
}

/*!
 * \brief Initializes DIO IRQ handlers
 *
 * \param [IN] irqHandlers Array containing the IRQ callback functions
 */
void SX126xIoIrqInit( DioIrqHandler dioIrq )
{
    gpio_config_t io_conf = {
        (1ULL<<lora_spi.irq_dio1),
        GPIO_MODE_INPUT,
        GPIO_PULLUP_DISABLE,
        GPIO_PULLDOWN_ENABLE,
        GPIO_INTR_POSEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_set_intr_type(lora_spi.irq_dio1, GPIO_INTR_POSEDGE));
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    ESP_ERROR_CHECK(gpio_isr_handler_add(lora_spi.irq_dio1, dioIrq, (void*) lora_spi.irq_dio1));
}

/*!
 * \brief Gets current state of DIO1 pin state.
 *
 * \retval state DIO1 pin current state.
 */
uint32_t SX126xGetDio1PinState( void )
{
    return gpio_get_level(lora_spi.irq_dio1);
}

/*!
 * \brief Initializes RF switch control pins.
 */
void SX126xIoRfSwitchInit( void )
{
    SX126xSetDio2AsRfSwitchCtrl(true);
}

/*!
 * \brief Initializes the TCXO power pin.
 */
void SX126xIoTcxoInit( void )
{
    // No TCXO component available on this board design.
}

/*!
 * \brief Sets the radio output power.
 *
 * \param [IN] power Sets the RF output power
 */
void SX126xSetRfTxPower( int8_t power )
{
    SX126xSetTxParams( power, RADIO_RAMP_40_US );
}

// SPI stuff

/*!
 * \brief Write a single byte of data to the radio memory
 *
 * \param [in]  address       The address of the first byte to write in the radio
 * \param [in]  value         The data to be written in radio's memory
 */
void SX126xWriteRegister( uint16_t address, uint8_t value )
{
    SX126xWriteRegisters(address, &value, 1);
}

/*!
 * \brief Read a single byte of data from the radio memory
 *
 * \param [in]  address       The address of the first byte to write in the radio
 *
 * \retval      value         The value of the byte at the given address in radio's memory
 */
uint8_t SX126xReadRegister( uint16_t address )
{
    uint8_t data;
    SX126xReadRegisters(address, &data, 1);
    return data;
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    spi_transaction_t trans = {
        .cmd = RADIO_READ_REGISTER,
        .addr = address,
        .rx_buffer = buffer,
        .length = size * 8  // size is in bits, not bytes
                            // https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/spi_master.html#_CPPv4N17spi_transaction_t6lengthE
    };

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xCheckDeviceReady();
#endif

    ESP_ERROR_CHECK(spi_device_transmit(lora_spi.handle, &trans));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xWaitOnBusy();
#endif
}
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    spi_transaction_t trans = {
        .cmd = RADIO_WRITE_REGISTER,
        .addr = address,
        .tx_buffer = buffer,
        .length = size * 8
    };

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xCheckDeviceReady();
#endif

    ESP_ERROR_CHECK(spi_device_transmit(lora_spi.handle, &trans));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xWaitOnBusy();
#endif
}

/*!
 * \brief Send a command that write data to the radio
 *
 * \param [in]  opcode        Opcode of the command
 * \param [in]  buffer        Buffer to be send to the radio
 * \param [in]  size          Size of the buffer to send
 */
void SX126xWriteCommand( RadioCommands_t opcode, uint8_t *buffer, uint16_t size )
{
    spi_transaction_ext_t trans = {
        .base = {
            .cmd = opcode,
            .tx_buffer = buffer,
            .length = size * 8,
            .flags = SPI_TRANS_VARIABLE_ADDR,
        },
        .address_bits = SX126X_NUM_COMMAND_ADDRESS_BITS
    };

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xCheckDeviceReady();
#endif

    ESP_ERROR_CHECK(spi_device_transmit(lora_spi.handle, (spi_transaction_t *)&trans));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xWaitOnBusy();
#endif
}

/*!
 * \brief Send a command that read data from the radio
 *
 * \param [in]  opcode        Opcode of the command
 * \param [out] buffer        Buffer holding data from the radio
 * \param [in]  size          Size of the buffer
 *
 * \retval status Return command radio status
 */
uint8_t SX126xReadCommand( RadioCommands_t opcode, uint8_t *buffer, uint16_t size )
{
    spi_transaction_ext_t trans = {
        .base = {
            .cmd = opcode,
            .rx_buffer = buffer,
            .length = size * 8,
            .flags = SPI_TRANS_VARIABLE_ADDR,
        },
        .address_bits = SX126X_NUM_COMMAND_ADDRESS_BITS
    };

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xCheckDeviceReady();
#endif

    ESP_ERROR_CHECK(spi_device_transmit(lora_spi.handle, (spi_transaction_t *)&trans));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xWaitOnBusy();
#endif

    return 0;
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    spi_transaction_ext_t trans = {
        .base = {
            .cmd = RADIO_READ_BUFFER,
            .addr = offset,
            .rx_buffer = buffer,
            .length = size * 8,
            .flags = SPI_TRANS_VARIABLE_ADDR,
        },
        .address_bits = SX126X_NUM_BUFFER_OFFSET_BITS
    };

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xCheckDeviceReady();
#endif

    ESP_ERROR_CHECK(spi_device_transmit(lora_spi.handle, (spi_transaction_t *)&trans));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xWaitOnBusy();
#endif
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
    spi_transaction_ext_t trans = {
        .base = {
            .cmd = RADIO_WRITE_BUFFER,
            .addr = offset,
            .tx_buffer = buffer,
            .length = size * 8,
            .flags = SPI_TRANS_VARIABLE_ADDR,
        },
        .address_bits = SX126X_NUM_BUFFER_OFFSET_BITS
    };

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xCheckDeviceReady();
#endif

    ESP_ERROR_CHECK(spi_device_transmit(lora_spi.handle, (spi_transaction_t *)&trans));

#if defined(SX1261MBXBAS) || defined(SX1262MBXCAS) || defined(SX1262MBXDAS)
    SX126xWaitOnBusy();
#endif
}

