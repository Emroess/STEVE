#!/bin/bash
#
# check_layering.sh - Hardware access layering validation
#
# This script checks that only designated files access STM32 hardware
# registers and headers, enforcing a layering architecture.
# Called during build to prevent accidental hardware access from
# application or middleware layers.
# - Whitelist approach (default deny)
# - Explicit allow for hardware layer files
# - Catches application code accessing hardware registers
#
# Exit code: 0 = pass, non-zero = violations found

set -e

# Files explicitly allowed to access hardware registers
ALLOWED_HW_ACCESS=(
	"board.c"
	"system_stm32h7xx.c"
	"uart.c"
	"fdcan.c"
	"stm32h7xx_it.c"
)

violations=0

echo "=== Hardware Layering Check ==="
echo "Checking that only whitelisted files access STM32 hardware..."
echo ""

# Check every .c file in src/
for file in src/*.c; do
	filename=$(basename "$file")
	
	# Is this file on the whitelist?
	allowed=0
	for allowed_file in "${ALLOWED_HW_ACCESS[@]}"; do
		if [ "$filename" == "$allowed_file" ]; then
			allowed=1
			break
		fi
	done
	
	# If NOT on whitelist, must not include stm32 hardware headers
	if [ $allowed -eq 0 ]; then
		if grep -qE '#include.*stm32(h7xx|_wrapper)\.h' "$file" 2>/dev/null; then
			echo "ERROR: $filename includes STM32 headers (not on whitelist)"
			violations=$((violations + 1))
		fi
		
		# Must not access hardware registers directly
		if grep -qE '(RCC|GPIO[A-K]|NVIC|SCB|MPU|USART|FDCAN|PWR|FLASH)->' "$file" 2>/dev/null; then
			echo "ERROR: $filename accesses hardware registers (not on whitelist)"
			violations=$((violations + 1))
		fi
	else
		echo "✓ $filename (on whitelist)"
	fi
done

echo ""
if [ $violations -eq 0 ]; then
	echo "✓ Layering check passed - no violations found"
	exit 0
else
	echo "✗ Layering check FAILED - $violations violation(s) found"
	exit 1
fi
