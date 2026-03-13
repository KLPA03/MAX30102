idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES led_strip nvs_flash
    PRIV_REQUIRES esp_timer esp_partition wear_levelling driver fatfs vfs
)

