#!/bin/bash
. /home/martin/esp/esp-idf/export.sh > /dev/null 2>&1
idf.py -p /dev/ttyACM0 flash
