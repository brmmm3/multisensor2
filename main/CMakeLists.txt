set(EXTRA_COMPONENT_DIRS "components")
idf_component_register(
    SRCS MultiSensor2.c config.c tcp_server.c console/console.c console/debug_cmd.c console/led_cmd.c console/pwr_cmd.c console/bme_cmd.c console/mhz19_cmd.c console/scd4x_cmd.c console/sps30_cmd.c console/wifi_cmd.c console/rtc_cmd.c console/ftp_cmd.c console/tcp_cmd.c console/config_cmd.c console/reset_cmd.c ui/ui_init.c ui/ui_config.c ui/ui_update.c wifi/wifi.c wifi/wifi_scan.c wifi/wifi_sntp.c
    INCLUDE_DIRS "."
    PRIV_REQUIRES nvs_flash esp_timer mbedtls esp_wifi console rtc_tiny
    REQUIRES ftp hw_serial lcd sdcard gps bmx280 scd4x mhz19 sps30 adxl345 qmc5883l)
