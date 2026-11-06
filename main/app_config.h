#pragma once

#include "driver/uart.h"

#if __has_include("secrets.h")
#include "secrets.h"
#endif

#ifndef EMU_WIFI_SSID
#define EMU_WIFI_SSID ""
#endif
#ifndef EMU_WIFI_PASSWORD
#define EMU_WIFI_PASSWORD ""
#endif
#ifndef EMU_WEB_PASSWORD
#define EMU_WEB_PASSWORD ""
#endif
#ifndef EMU_AP_PASSWORD
#define EMU_AP_PASSWORD "modbus-setup"
#endif

// See docs/wiring.svg. UART0 stays on USB for flashing and logs.
#define EMU_UART_PORT     UART_NUM_2
#define EMU_PIN_UART_TX   17
#define EMU_PIN_UART_RX   16
#define EMU_PIN_LED_TX    25
#define EMU_PIN_LED_RX    26
#define EMU_PIN_LED_WIFI  27

#define EMU_HOSTNAME      "modbus-emu"
#define EMU_AP_SSID       "Modbus-Emu-Setup"

// Serial defaults, all adjustable from the web UI
#define EMU_DEFAULT_BAUD   57600
#define EMU_DEFAULT_PARITY 'N'
#define EMU_DEFAULT_STOP   1
#define EMU_DEFAULT_SLAVE  1
