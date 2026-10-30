# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(rp2040_usb_webserver C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()
find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(WEB_ASSETS_HEADER ${CMAKE_CURRENT_BINARY_DIR}/generated/web_assets.h)
add_custom_command(
    OUTPUT ${WEB_ASSETS_HEADER}
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_LIST_DIR}/tools/embed_web.py
        ${CMAKE_CURRENT_LIST_DIR}/web ${WEB_ASSETS_HEADER}
    DEPENDS tools/embed_web.py web/index.html web/app.js web/gpio-model.js
        web/pin-store.js web/pico-pins.js web/pin-session.js web/gpio-transport.js web/style.css
    VERBATIM
)

# Add executable. Default name is the project name, version 0.1

add_executable(rp2040_usb_webserver
    main.c
    gpio_control.c
    gpio_api.c
    system_control.c
    captive_portal.c
    network_status.c
    ${WEB_ASSETS_HEADER}
    web_fs.c
    usb_network.c
    usb_descriptors.c
    dhcpserver/dhcpserver.c
    dhcpserver/dhcp_options.c
    ${PICO_TINYUSB_PATH}/lib/networking/rndis_reports.c
)

pico_set_program_name(rp2040_usb_webserver "rp2040_usb_webserver")
pico_set_program_version(rp2040_usb_webserver "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(rp2040_usb_webserver 0)
pico_enable_stdio_usb(rp2040_usb_webserver 0)

# Add libraries to the build
target_link_libraries(rp2040_usb_webserver
    pico_stdlib
    pico_bootrom
    hardware_pwm
    hardware_clocks
    hardware_adc
    hardware_watchdog
    tinyusb_device
    pico_lwip
    pico_lwip_nosys
    pico_lwip_http
    pico_lwip_mdns
    pico_unique_id
)

# Add include directories to the build
target_include_directories(rp2040_usb_webserver PRIVATE
    ${PICO_TINYUSB_PATH}/lib/networking # for rndis_protocol.h
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}/generated
)

pico_add_extra_outputs(rp2040_usb_webserver)
