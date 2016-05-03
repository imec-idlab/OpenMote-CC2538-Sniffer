# Copy sniffer to root directory after building
COPY_SNIFFER_HEX = TRUE

# Project name and files to compile
PROJECT_NAME  = sniffer
PROJECT_FILES = main.cpp sniffer_global.cpp sniffer_radio.cpp sniffer_serial.cpp sniffer_serial_send.cpp sniffer_serial_receive.cpp
PROJECT_DIR   = .

# Location of the root directory
PROJECT_HOME = ../OpenMoteFirmware

# Include the current path
INC_PATH += -I $(PROJECT_DIR)

# Define options passed to the C compiler
DOPTIONS += -DNO_CLOCK_DIVIDER_RESTORE

# Configure compiling
USE_BOARD = TRUE
USE_DRIVERS = TRUE
USE_KERNEL = TRUE
USE_LIBRARY = TRUE
USE_PLATFORM = TRUE

# Hardcode BOARD and TARGET variables
TARGET = cc2538
BOARD  = openmote-cc2538

# Include the Makefile in the root directory
include $(PROJECT_HOME)/Makefile.include
