set(EXTRA_COMPONENT_DIRS "components")
idf_component_register(
    SRCS "MultiSensor2.c" "lvgl_demo_ui.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer esp_netif fatfs sd_card
    REQUIRES wifi ftp hw_serial sw_serial lcd gps bmx280 scd4x mhz19 sps30 yys)
