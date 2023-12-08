idf_component_register(SRCS "sht31_driver.c"
                    INCLUDE_DIRS "include"
                    REQUIRES driver
                    REQUIRES i2c_interface)

