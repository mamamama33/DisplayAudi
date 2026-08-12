idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES PhysicalCan TP2.0Protocol KwpProtocol DataAnaliser Display
)