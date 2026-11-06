// Copy to secrets.h (gitignored) and fill in. Every value is optional:
// credentials saved from the web UI take priority over these.
#pragma once

#define EMU_WIFI_SSID     "your-network"
#define EMU_WIFI_PASSWORD "your-password"

// Required for changes made from the web UI. Empty means no password.
#define EMU_WEB_PASSWORD  ""

// WPA2 password for the fallback setup network, at least 8 characters.
#define EMU_AP_PASSWORD   "modbus-setup"
