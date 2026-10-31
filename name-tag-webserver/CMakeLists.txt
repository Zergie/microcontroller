cmake_minimum_required(VERSION 3.13)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_BOARD pico CACHE STRING "Board SDK configuration")
include(../shared/rp2040-usb-web/pico_sdk_import.cmake)
project(name_tag_webserver C CXX ASM)
pico_sdk_init()
include(../shared/rp2040-usb-web/CMakeLists.txt)
add_executable(name_tag_webserver main.c
    animations/effects.c animations/lab_effects.c animations/animation_engine.c
    api/system_api.c api/animation_api.c api/pads_api.c api/web_routes.c pads/pads.c
    pads/ws_protocol.c pads/ws_server.c pixels/pixels.c
    settings/settings.c settings/flash_storage.c)
target_include_directories(name_tag_webserver PRIVATE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(name_tag_webserver hardware_pio hardware_dma hardware_clocks hardware_flash pico_flash pico_rand)
# This firmware never launches core 1. Interrupts are masked by flash_safe_execute.
target_compile_definitions(name_tag_webserver PRIVATE PICO_FLASH_ASSUME_CORE1_SAFE=1)
target_link_options(name_tag_webserver PRIVATE "LINKER:--script=${CMAKE_CURRENT_LIST_DIR}/settings/reserve.ld")
set_property(TARGET name_tag_webserver APPEND PROPERTY LINK_DEPENDS "${CMAKE_CURRENT_LIST_DIR}/settings/reserve.ld")
pico_generate_pio_header(name_tag_webserver ${CMAKE_CURRENT_LIST_DIR}/pixels/ws2812.pio)
target_usb_web(name_tag_webserver "${CMAKE_CURRENT_LIST_DIR}/web")
pico_set_program_name(name_tag_webserver "name_tag_webserver")
pico_set_program_version(name_tag_webserver "0.1")
