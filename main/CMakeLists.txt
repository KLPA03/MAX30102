set(srcs
    "tusb_msc_main.c"
    "max30101.c"
)

set(requires
    console
    driver
    esp_partition
    esp_timer
    fatfs
    wear_levelling
)

# Headers for TinyUSB MSC live in the esp_tinyusb managed component.
list(APPEND requires esp_tinyusb)

idf_component_register(
    SRCS "${srcs}"
    INCLUDE_DIRS .
    REQUIRES "${requires}"
)

