idf_component_register(SRCS "rs485_example.cpp"
                            "unitree_go_motor.c"
                            "crc_utils.cpp"
                    REQUIRES nvs_flash esp_driver_uart esp_driver_gpio
                    INCLUDE_DIRS ".")
