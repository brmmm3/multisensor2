set(EXTRA_COMPONENT_DIRS "components")
idf_component_register(
    SRCS "MultiSensor2.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer
    REQUIRES wifi rtc_tiny ftp hw_serial sw_serial lcd ui sdcard gps bmx280 scd4x mhz19 sps30 yys adxl345 tlv493)
