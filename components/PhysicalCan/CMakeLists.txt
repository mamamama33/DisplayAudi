idf_component_register(SRCS "PhysicalCan.c"
    PRIV_REQUIRES esp_driver_gpio esp_driver_twai driver
                            
    PRIV_REQUIRES TP2.0Protocol 
    PRIV_REQUIRES KwpProtocol
    
                    INCLUDE_DIRS ".")
