#!/bin/bash
# Sačuvaj trenutni folder
CURRENT_DIR=$(pwd)

# Idi do ESP-IDF i eksportuj
cd ~/Desktop/esp/esp-idf
source export.sh

# Vrati se u originalni folder
cd "$CURRENT_DIR"

# Reci gde si
echo "----------------------------------------"
echo "Vratio si se u: $CURRENT_DIR"
echo "Spreman za rad! Koristi: idf.py build"
echo "----------------------------------------"
