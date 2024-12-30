set(EXTRA_COMPONENT_DIRS "components")
idf_component_register(
    SRCS "MultiSensor2.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES "esp_timer"
    REQUIRES "hw_serial" "sw_serial" "gps" "lcd" "bmx280" "scd4x" "sps30" "yys")
