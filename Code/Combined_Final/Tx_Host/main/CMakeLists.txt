idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    REQUIRES usb_host_msc usb_host_hid driver)
# Notes:
# - usb_host_msc and usb_host_hid refer to the installed components
#   (either built-in IDF or managed components).
# - driver is needed for UART/GPIO.