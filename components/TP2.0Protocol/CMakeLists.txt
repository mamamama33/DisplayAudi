idf_component_register(
    SRCS "TP2.0Protocol.c"
    PRIV_REQUIRES KwpProtocol
    PRIV_REQUIRES PhysicalCan
    INCLUDE_DIRS "."
)