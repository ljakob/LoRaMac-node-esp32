# LoRaMac-node-esp32
#### ESP32 port of LoRaMac-node
Creates a Semtech SX126x compatible ESP32 component that pulls in all LoRaMac-node sources, allowing for a custom ESP32 board definition to be used.

## Compatibility
This library is written for and is currently only compatible with Semtech SX126x LoRaWAN chips.

## Configuration
- Running menuconfig with LoRaMac-node-esp32 added as a component will allow for configuration of SPI pins and the host to be used.
- Region(s) of operation must be selected by pulling in one of the pre-processor definitions used by LoRaMac-node, e.g. `REGION_US915`.
  - Regions are listed in `src/LoRaMac-node/src/mac/region/Region.h`

## Notes
- `SX126xWaitOnBusy()` indefinitely blocks with a busy wait, waiting for a low BUSY signal 
- This is work in progress - not usable yet
