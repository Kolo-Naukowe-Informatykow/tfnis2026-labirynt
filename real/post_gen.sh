#!/bin/bash
PLATFORM_FILE="Drivers/BSP/Components/vl53l4cd/porting/platform.c"

if [ -f "$PLATFORM_FILE" ]; then
    sed -i 's/^uint8_t VL53L4CD_WaitMs/uint8_t __attribute__((weak)) VL53L4CD_WaitMs/' "$PLATFORM_FILE"
    echo "✓ Patched VL53L4CD_WaitMs with __attribute__((weak))"
else
    echo "✗ Platform file not found: $PLATFORM_FILE"
fi
