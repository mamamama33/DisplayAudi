idf_component_register(
    SRCS "PhysicalCan.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES driver TP2.0Protocol KwpProtocol
)
