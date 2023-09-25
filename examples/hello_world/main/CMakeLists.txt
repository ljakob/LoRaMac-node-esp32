idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES LoRaMac-node-esp32)

add_definitions(-DACTIVE_REGION=LORAMAC_REGION_EU868)
add_definitions(-DLORAWAN_PUBLIC_NETWORK=true)
