idf_component_register(SRCS "app_main.c"
                            "emu_map.c"
                            "leds.c"
                            "map_store.c"
                            "mb_frame.c"
                            "modbus_slave.c"
                            "settings.c"
                            "web.c"
                            "wifi.c"
                            "wifi_led.c"
                       INCLUDE_DIRS "."
                       EMBED_TXTFILES "web/index.html"
                       PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_event esp_http_server
                                     esp_netif esp_timer esp_wifi json nvs_flash)
