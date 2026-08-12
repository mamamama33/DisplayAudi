idf_component_register(
    SRCS "Display.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        DataAnaliser 
        PhysicalCan 
        arduino 
        TFT_eSPI 
        AnimatedGIF
)