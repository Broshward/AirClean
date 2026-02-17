idf_component_register(SRCS "blufi_main.c"
                            "blufi_security.c"
							"oneshot_read_adc_main.c"
                            "blufi_init.c"
							"tasks.c"
                    PRIV_REQUIRES bt nvs_flash mbedtls esp_driver_gpio lwip esp_adc esp_driver_i2c
                    INCLUDE_DIRS ".")
