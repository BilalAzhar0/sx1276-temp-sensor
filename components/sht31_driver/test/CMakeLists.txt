idf_component_register(SRC_DIRS "."
                    INCLUDE_DIRS "."
                    REQUIRES driver i2c_interface
                    REQUIRES cmock sht31_driver)
