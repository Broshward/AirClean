idf_component_register(SRCS "blufi_main.c"
                            "blufi_security.c"
							"oneshot_read_adc_main.c"
							#"adc.c"
                            "blufi_init.c"
							"tasks.c"
							"times.c"
							"ping.c"
							"i2c.c"
							"spi.c"
							"ota.c"
							"remote_log.c"
							"sensor.c"
							"gatts.c"
                    PRIV_REQUIRES bt nvs_flash mbedtls esp_driver_gpio lwip esp_adc esp_driver_i2c esp_driver_tsens esp_driver_spi esp_https_ota mbedtls app_update

                    INCLUDE_DIRS ".")


