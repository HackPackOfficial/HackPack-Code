# Define the CMake project for the NHY3274TH sensor library
idf_component_register(
    SRCS "NHY3274TH.cpp"             # Source file(s)
    INCLUDE_DIRS "."                 # Include directory (current directory in this case)
    REQUIRES Arduino Adafruit_BusIO  # Dependencies required by the library
)