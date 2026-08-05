idf_component_register(
    SRCS "Display.cpp"
    PRIV_REQUIRES DataAnaliser
    PRIV_REQUIRES PhysicalCan 
    PRIV_REQUIRES TFT_eSPI 
    PRIV_REQUIRES AnimatedGIF
    INCLUDE_DIRS "."

)

