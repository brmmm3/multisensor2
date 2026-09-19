#!/bin/bash
. /home/martin/esp/esp-idf/export.sh > /dev/null 2>&1
# idf.py add-dependency "espressif/led_strip^3.0.0"
idf.py add-dependency $0
