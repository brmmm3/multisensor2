set(EXTRA_COMPONENT_DIRS "components")
idf_component_register(
    SRCS "MultiSensor2.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "esp_timer"
    REQUIRES "hw_serial" "sw_serial" "lcd" "gps" "bmx280" "scd4x" "mhz19" "sps30" "yys")
