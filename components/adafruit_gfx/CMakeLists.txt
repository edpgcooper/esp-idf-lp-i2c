# Adafruit GFX Library
# https://github.com/adafruit/Adafruit-GFX-Library
# BSD License

cmake_minimum_required(VERSION 3.5)

idf_component_register(SRCS "Adafruit_GFX.cpp" "glcdfont.c" "Print.cpp"
                       INCLUDE_DIRS ".")
# REQUIRES arduino Adafruit_BusIO
#  "Adafruit_GrayOLED.cpp" "Adafruit_SPITFT.cpp"
#project(Adafruit-GFX-Library)

