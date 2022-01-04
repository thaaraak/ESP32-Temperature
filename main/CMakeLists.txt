# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.cpp" "I2CDevice.cpp" "Adafruit_ADS1X15.cpp" "Adafruit_MAX31865.cpp")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
