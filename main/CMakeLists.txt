idf_component_register(SRCS "main.c"
                            "sx127x.c"
                            "sht3x.c"
                            "bmp280.c"
                            "bh1750.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES esp_driver_i2c
                                     esp_driver_spi
                                     esp_driver_gpio)
