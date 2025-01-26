set(EXTRA_COMPONENT_DIRS "components")
idf_component_register(
    SRCS MultiSensor2.c wifi_sntp.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_timer console rtc_tiny
    REQUIRES wifi ftp hw_serial sw_serial lcd ui sdcard gps bmx280 scd4x mhz19 sps30 yys adxl345 tlv493 qmc5883l)
