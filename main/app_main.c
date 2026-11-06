#include "modbus_slave.h"
#include "map_store.h"
#include "settings.h"
#include "web.h"
#include "wifi.h"

void app_main(void) {
    serial_cfg_t serial;

    settings_init();
    settings_load_serial(&serial);

    // Modbus comes up first and never depends on the network
    store_init();
    modbus_slave_start(&serial);

    wifi_start();
    web_start();
}
