#!/usr/bin/env bash
# Produce an Embassy-friendly libgraphics4d_rp2350.a from the vendored Workshop5
# archive: ARM-only members, without Pico app startup / USB stdio / newlib glue that
# clashes with cortex-m-rt link.x.
#
# Usage: filter-graphics4d-embassy-lib.sh INPUT.a OUTPUT.a

set -euo pipefail

in="${1:?input .a}"
out="${2:?output .a}"

tmpdir="$(mktemp -d)"
trap 'rm -rf "${tmpdir}"' EXIT

cd "${tmpdir}"
ar x "${in}"

drop_object() {
    case "$1" in
        # Pico app startup / heap glue (Embassy provides its own)
        newlib_interface.c.o|runtime_init.c.o|runtime_init_clocks.c.o|runtime_init_stack_guard.c.o|panic.c.o|mutex.c.o|standard_binary_info.c.o|runtime.c.o|platform.c.o)
            return 0
            ;;
        # Embassy USB logger — not Pico tinyusb stdio
        stdio_usb.c.o|stdio_usb_descriptors.c.o|stdio.c.o|stdio_uart.c.o)
            return 0
            ;;
        # Host-only / unrelated USB device stack for this LVGL demo
        audio_device.c.o|cdc_device.c.o|dfu_device.c.o|dfu_rt_device.c.o|hid_device.c.o|midi_device.c.o|msc_device.c.o|ecm_rndis_device.c.o|ncm_device.c.o|usbtmc_device.c.o|vendor_device.c.o|video_device.c.o|tusb.c.o|tusb_fifo.c.o|usbd.c.o|usbd_control.c.o|dcd_rp2040.c.o|rp2040_usb.c.o|rp2040_usb_device_enumeration.c.o|reset_interface.c.o)
            return 0
            ;;
    esac
    return 1
}

rm -f "${out}"
count=0
for obj in *.o; do
    [[ -f "${obj}" ]] || continue
    if drop_object "${obj}"; then
        continue
    fi
    if ! readelf -h "${obj}" 2>/dev/null | grep -q 'Machine:.*ARM'; then
        continue
    fi
    arm-none-eabi-ar q "${out}" "${obj}"
    count=$((count + 1))
done

[[ "${count}" -gt 0 ]] || {
    echo "filter-graphics4d-embassy-lib: no objects kept from ${in}" >&2
    exit 1
}

echo "filter-graphics4d-embassy-lib: kept ${count} object(s) -> ${out}" >&2
