idf_component_register(
    SRCS "src/AnimatedGIF.cpp"  # ili kako su izlistani .cpp fajlovi
    INCLUDE_DIRS "src"
    PRIV_REQUIRES esp_timer      # <--- OVA LINIJA DODAJE PUTANJU ZA esp_timer.h
)