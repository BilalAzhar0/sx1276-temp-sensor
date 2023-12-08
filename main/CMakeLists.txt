idf_component_register(SRCS "sx1276-esp32.c"
            INCLUDE_DIRS .
            INCLUDE_DIRS "../components/sht31_driver/include"
		    INCLUDE_DIRS "../components/i2c_interface/include"
			INCLUDE_DIRS "../components/lora/include"
			INCLUDE_DIRS "../components/lora"
		    REQUIRES i2c_interface
		    REQUIRES sht31_driver
		    REQUIRES nvs_flash
		    )
