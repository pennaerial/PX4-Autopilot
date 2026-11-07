# LittleFS does not provide file modification times in this NuttX version.
if(NOT "${PX4_BOARD_LABEL}" STREQUAL "bootloader")
    add_compile_definitions(KAKUTEH7MINI_FLASH_LOG_TIMESTAMPS)
endif()
