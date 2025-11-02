#!/bin/bash
# ==========================================================
# Script: check_serial_port.sh
# ==========================================================

# obtain list of serial devices
serial_devices=(/dev/serial/by-id/*)

# check if directory exists and is not empty
if [ ! -d "/dev/serial/by-id" ] || [ ${#serial_devices[@]} -eq 0 ]; then
    echo "====== [ERROR] No serial devices found under /dev/serial/by-id/ ======"
    exit 1
fi

# obtain device count
count=${#serial_devices[@]}

# check count
if [ $count -eq 1 ]; then
    echo "====== [INFO] Found 1 serial device ======"
    echo "Device path: ${serial_devices[0]}"
    exit 0
else
    echo "====== [WARNING] Found $count serial devices ======"
    echo "List of devices:"
    for dev in "${serial_devices[@]}"; do
        echo "  - $dev"
    done
    echo "====== Please ensure only ONE device is connected before running this script. ======"
    exit 1
fi
