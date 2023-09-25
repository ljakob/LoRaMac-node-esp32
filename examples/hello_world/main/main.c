/**
 * minimalistic example with OTAA
 * based using an RFM95W (SX1276)
 */

#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "esp_err.h"
#include "esp_log.h"

// includes for lora stack
#include "RegionCommon.h"
#include "LmHandler.h"
#include "LmhpCompliance.h"
#include "LmhpClockSync.h"

#include "board.h"

static const char *TAG = "HelloWorld";

// SAME FOR ALL BOARDS

#ifndef ACTIVE_REGION

#warning "No active region defined, LORAMAC_REGION_EU868 will be used as default."

#define ACTIVE_REGION LORAMAC_REGION_EU868

#endif

/*!
 * LoRaWAN default end-device class
 */
#define LORAWAN_DEFAULT_CLASS                       CLASS_A

/*!
 * Defines the application data transmission duty cycle. 40s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            40000

/*!
 * Defines a random delay for application data transmission duty cycle. 5s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        5000

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_STATE                           LORAMAC_HANDLER_ADR_ON

/*!
 * Default datarate
 *
 * \remark Please note that LORAWAN_DEFAULT_DATARATE is used only when ADR is disabled 
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_3

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_DEFAULT_CONFIRMED_MSG_STATE         LORAMAC_HANDLER_UNCONFIRMED_MSG

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_BUFFER_MAX_SIZE            242

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true
//

static uint8_t AppDataBuffer[LORAWAN_APP_DATA_BUFFER_MAX_SIZE];

/*
 * MC Session Started
 */
static volatile bool IsMcSessionStarted = false;

// prototypes
static void OnClassChange( DeviceClass_t deviceClass );

static LmHandlerCallbacks_t LmHandlerCallbacks =
{
    .GetBatteryLevel = NULL,
    .GetTemperature = NULL,
    .GetRandomSeed = NULL,
    .OnMacProcess = NULL,
    .OnNvmDataChange = NULL,
    .OnNetworkParametersChange = NULL,
    .OnMacMcpsRequest = NULL,
    .OnMacMlmeRequest = NULL,
    .OnJoinRequest = NULL,
    .OnTxData = NULL,
    .OnRxData = NULL,
    .OnClassChange= OnClassChange,
    .OnBeaconStatusChange = NULL,
    .OnSysTimeUpdate = NULL,
};

static LmHandlerParams_t LmHandlerParams =
{
    .Region = ACTIVE_REGION,
    .AdrEnable = LORAWAN_ADR_STATE,
    .IsTxConfirmed = LORAWAN_DEFAULT_CONFIRMED_MSG_STATE,
    .TxDatarate = LORAWAN_DEFAULT_DATARATE,
    .PublicNetworkEnable = LORAWAN_PUBLIC_NETWORK,
    .DutyCycleEnabled = LORAWAN_DUTYCYCLE_ON,
    .DataBufferMaxSize = LORAWAN_APP_DATA_BUFFER_MAX_SIZE,
    .DataBuffer = AppDataBuffer,
    .PingSlotPeriodicity = REGION_COMMON_DEFAULT_PING_SLOT_PERIODICITY,
};

static LmhpComplianceParams_t LmhpComplianceParams =
{
    .FwVersion.Value = 42, // FIXME
    .OnTxPeriodicityChanged = NULL,
    .OnTxFrameCtrlChanged = NULL,
    .OnPingSlotPeriodicityChanged = NULL,
};

static void OnClassChange( DeviceClass_t deviceClass )
{
    ESP_LOGI(TAG, "OnClassChange class=%i", (int)deviceClass);

    switch( deviceClass )
    {
        default:
        case CLASS_A:
        {
            IsMcSessionStarted = false;
            break;
        }
        case CLASS_B:
        {
            // Inform the server as soon as possible that the end-device has switched to ClassB
            LmHandlerAppData_t appData =
            {
                .Buffer = NULL,
                .BufferSize = 0,
                .Port = 0,
            };
            LmHandlerSend( &appData, LORAMAC_HANDLER_UNCONFIRMED_MSG );
            IsMcSessionStarted = true;
            break;
        }
        case CLASS_C:
        {
            IsMcSessionStarted = true;
            break;
        }
    }
}

static bool FIRST_MESSAGE_SENT = false;

static void UplinkProcess( void )
{
    LmHandlerErrorStatus_t status = LORAMAC_HANDLER_ERROR;

    if ( FIRST_MESSAGE_SENT )
    {
        return;
    }

    if( LmHandlerIsBusy( ) == true )
    {
        return;
    }

    uint8_t isPending = 1;
    if( isPending == 1 )
    {
        if( IsMcSessionStarted == false )
        {
            AppDataBuffer[0] = 'a';
            AppDataBuffer[1] = 'b';
            AppDataBuffer[2] = 'c';
            AppDataBuffer[3] = 'd';
            AppDataBuffer[4] = 'e';

            // Send FragAuthReq
            LmHandlerAppData_t appData =
            {
                .Buffer = AppDataBuffer,
                .BufferSize = 5,
                .Port = 1,
            };
            status = LmHandlerSend( &appData, LmHandlerParams.IsTxConfirmed );
            FIRST_MESSAGE_SENT = true;

            if( status == LORAMAC_HANDLER_SUCCESS )
            {
                ESP_LOGI(TAG, "message sent");
            }
            else
            {
                ESP_LOGW(TAG, "message NOT sent");
            }
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Hello World");

    BoardInitMcu( );
    BoardInitPeriph( );

    if ( LmHandlerInit( &LmHandlerCallbacks, &LmHandlerParams ) != LORAMAC_HANDLER_SUCCESS )
    {
        ESP_LOGE(TAG, "LoRaMac wasn't properly initialized\n" );
        // Fatal error, endless loop.
        while ( 1 )
        {
        }
    }

    // Set system maximum tolerated rx error in milliseconds
    LmHandlerSetSystemMaxRxError( 20 );

    // The LoRa-Alliance Compliance protocol package should always be
    // initialized and activated.
    LmHandlerPackageRegister( PACKAGE_ID_COMPLIANCE, &LmhpComplianceParams );
    LmHandlerPackageRegister( PACKAGE_ID_CLOCK_SYNC, NULL );
    //LmHandlerPackageRegister( PACKAGE_ID_REMOTE_MCAST_SETUP, NULL );
    //LmHandlerPackageRegister( PACKAGE_ID_FRAGMENTATION, &FragmentationParams );

    LmHandlerJoin( );

    while( 1 )
    {
        // Processes the LoRaMac events
        LmHandlerProcess( );

        // Process application uplinks management
        UplinkProcess( );

#if 0
        CRITICAL_SECTION_BEGIN( );
        if( IsMacProcessPending == 1 )
        {
            // Clear flag and prevent MCU to go into low power modes.
            IsMacProcessPending = 0;
        }
        else
        {
            // The MCU wakes up through events
            BoardLowPowerHandler( );
        }
        CRITICAL_SECTION_END( );
#endif
    }


}
