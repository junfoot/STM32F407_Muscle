# Archived Source Files

This folder contains source files that are kept for reference but are **not part of the active build**.

## Contents

- `USB_HOST/App/usbh_ch340.c`
- `USB_HOST/App/usbh_ch340.h`

## Reason for Archiving

The WT9011DCL-RF receiver currently enumerates as a USB CDC-ACM class device, so the firmware uses the standard STM32 USB Host CDC class driver (`usbh_cdc.c`) and the CDC state machine in `Core/Src/main.c`.

The CH340 vendor-class driver was an alternative implementation intended for the case where the receiver presents itself as a CH340 USB-to-UART bridge. It is preserved here so it can be restored quickly if the hardware configuration changes or if a CH340-based variant needs to be supported.

## How to Restore

1. Move `usbh_ch340.c` and `usbh_ch340.h` back to `USB_HOST/App/`.
2. In `USB_HOST/App/usb_host.c`, replace `USBH_RegisterClass(&hUsbHostFS, USBH_CDC_CLASS)` with `USBH_RegisterClass(&hUsbHostFS, USBH_CH340_CLASS)`.
3. Update `Core/Src/main.c` to use `USBH_CH340_Receive()` / `USBH_CH340_ReceiveCallback()` instead of the CDC-ACM equivalents.
