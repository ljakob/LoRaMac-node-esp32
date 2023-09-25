
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"

#include "freertos/task.h"

#include "board.h"

static const char *TAG = "ESP32Board";

static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

void BoardInitMcu( void )
{
}

void BoardInitPeriph(void)
{

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
    int res = nvs_open(namespace, NVS_READONLY, &handle);
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
    *milliseconds = 29;
    return  42 /* seconds */;
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
    return 0;
}

/*!
 * \brief Get the RTC timer value
 *
 * \retval RTC Timer value
 */
uint32_t RtcGetTimerValue( void )
{
    return 0;
}

/*!
 * \brief converts time in ms to time in ticks
 *
 * \param[IN] milliseconds Time in milliseconds
 * \retval returns time in timer ticks
 */
uint32_t RtcMs2Tick( TimerTime_t milliseconds )
{
    return 0;
}


/*!
 * \brief converts time in ticks to time in ms
 *
 * \param[IN] time in timer ticks
 * \retval returns time in milliseconds
 */
TimerTime_t RtcTick2Ms( uint32_t tick )
{
    return 0;
}


/*!
 * \brief Sets the RTC timer reference
 *
 * \retval value Timer reference value in ticks
 */
uint32_t RtcSetTimerContext( void )
{
    return 0;
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
    
}

/*!
 * \brief Stops the Alarm
 */
void RtcStopAlarm( void )
{
    
}



// RADIO

#include "sx126x-board.h"

#define BOARD_TCXO_WAKEUP_TIME                      0

static RadioOperatingModes_t OperatingMode;

/*!
 * \brief HW Reset of the radio
 */
void SX126xReset( void )
{

}

/*!
 * \brief Wakes up the radio
 */
void SX126xWakeup( void )
{

}


/*!
 * \brief De-initializes the RF Switch I/Os pins interface
 *
 * \remark Needed to decrease the power consumption in MCU low power modes
 */
void SX126xAntSwOff( void )
{

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
    return SX1262;
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
    //GpioSetInterrupt( &SX126x.DIO1, IRQ_RISING_EDGE, IRQ_HIGH_PRIORITY, dioIrq );

}

/*!
 * \brief Gets current state of DIO1 pin state.
 *
 * \retval state DIO1 pin current state.
 */
uint32_t SX126xGetDio1PinState( void )
{
    return 0;
}

/*!
 * \brief Initializes RF switch control pins.
 */
void SX126xIoRfSwitchInit( void )
{

}

/*!
 * \brief Initializes the TCXO power pin.
 */
void SX126xIoTcxoInit( void )
{

}

/*!
 * \brief Sets the radio output power.
 *
 * \param [IN] power Sets the RF output power
 */
void SX126xSetRfTxPower( int8_t power )
{

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
    return 0;    
}

void SX126xReadRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
    
}
void SX126xWriteRegisters( uint16_t address, uint8_t *buffer, uint16_t size )
{
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
    return 0;
}

void SX126xReadBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
}

void SX126xWriteBuffer( uint8_t offset, uint8_t *buffer, uint8_t size )
{
}

