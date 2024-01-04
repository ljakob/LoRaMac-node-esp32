set(COMPONENT_REQUIRES
    nvs_flash
    efuse
)



set(COMPONENT_SRCS 
"src/board/generic_esp32/generic-esp32-board.c"

"src/LoRaMac-node/src/mac/LoRaMacAdr.c"
"src/LoRaMac-node/src/mac/LoRaMac.c"
"src/LoRaMac-node/src/mac/LoRaMacClassB.c"
"src/LoRaMac-node/src/mac/LoRaMacCommands.c"
"src/LoRaMac-node/src/mac/LoRaMacConfirmQueue.c"
"src/LoRaMac-node/src/mac/LoRaMacCrypto.c"
"src/LoRaMac-node/src/mac/LoRaMacParser.c"
"src/LoRaMac-node/src/mac/LoRaMacSerializer.c"
"src/LoRaMac-node/src/system/timer.c"
"src/LoRaMac-node/src/system/systime.c"
"src/LoRaMac-node/src/system/nvmm.c"
"src/LoRaMac-node/src/radio/sx126x/radio.c"
"src/LoRaMac-node/src/radio/sx126x/sx126x.c"
"src/LoRaMac-node/src/radio/sx1276/sx1276.c"
"src/LoRaMac-node/src/boards/mcu/utilities.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/CayenneLpp.c"
"src/LoRaMac-node/src/peripherals/soft-se/soft-se.c"
"src/LoRaMac-node/src/peripherals/soft-se/soft-se-hal.c"
"src/LoRaMac-node/src/peripherals/soft-se/cmac.c"
"src/LoRaMac-node/src/peripherals/soft-se/aes.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/NvmDataMgmt.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/LmHandler.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/packages/FragDecoder.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/packages/LmhpFragmentation.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/packages/LmhpRemoteMcastSetup.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/packages/LmhpClockSync.c"
"src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/packages/LmhpCompliance.c"
"src/LoRaMac-node/src/mac/region/Region.c"
"src/LoRaMac-node/src/mac/region/RegionBaseUS.c"
"src/LoRaMac-node/src/mac/region/RegionCommon.c"
"src/LoRaMac-node/src/mac/region/RegionEU868.c"
"src/LoRaMac-node/src/mac/region/RegionUS915.c"
)

# idf.py build 2>&1 | tee LOG | grep "undefined reference" | sed -e 's/.*reference to//' | sort | uniq

set(COMPONENT_ADD_INCLUDEDIRS
    "src/LoRaMac-node/src/mac"
    "src/LoRaMac-node/src/mac/region"
    "src/LoRaMac-node/src/system"
    "src/LoRaMac-node/src/radio"
    "src/LoRaMac-node/src/boards"
    "src/LoRaMac-node/src/apps/LoRaMac/common"
    "src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler"
    "src/LoRaMac-node/src/apps/LoRaMac/common/LmHandler/packages"
    "src/LoRaMac-node/src/peripherals/soft-se"
)

idf_component_register(SRCS ${COMPONENT_SRCS}
                       INCLUDE_DIRS ${COMPONENT_ADD_INCLUDEDIRS}
                       REQUIRES ${COMPONENT_REQUIRES}
)

include_directories(
    $ENV{IDF_PATH}/components/driver/gpio/include
    $ENV{IDF_PATH}/components/driver/spi/include
    $ENV{IDF_PATH}/components/esp_timer/include
)
add_definitions(-DSOFT_SE)
