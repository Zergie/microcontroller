# Sources are compiled for each executable with that application's SDK config.
set(USB_WEB_ROOT "${CMAKE_CURRENT_LIST_DIR}")

function(target_usb_web target web_root)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    file(GLOB_RECURSE assets CONFIGURE_DEPENDS "${web_root}/*")
    set(header "${CMAKE_CURRENT_BINARY_DIR}/generated/web_assets.h")
    add_custom_command(OUTPUT "${header}"
        COMMAND ${Python3_EXECUTABLE} "${USB_WEB_ROOT}/tools/embed_web.py" "${web_root}" "${header}"
        DEPENDS "${USB_WEB_ROOT}/tools/embed_web.py" ${assets} VERBATIM)
    target_sources(${target} PRIVATE "${header}"
        "${USB_WEB_ROOT}/network_runtime.c" "${USB_WEB_ROOT}/web_fs.c"
        "${USB_WEB_ROOT}/usb_network.c" "${USB_WEB_ROOT}/usb_descriptors.c"
        "${USB_WEB_ROOT}/network_status.c" "${USB_WEB_ROOT}/system_control.c"
        "${USB_WEB_ROOT}/captive_portal.c"
        "${USB_WEB_ROOT}/dhcpserver/dhcpserver.c" "${USB_WEB_ROOT}/dhcpserver/dhcp_options.c"
        "${PICO_TINYUSB_PATH}/lib/networking/rndis_reports.c")
    target_include_directories(${target} PRIVATE "${USB_WEB_ROOT}"
        "${CMAKE_CURRENT_BINARY_DIR}/generated" "${PICO_TINYUSB_PATH}/lib/networking")
    target_link_libraries(${target} pico_stdlib pico_bootrom hardware_watchdog
        tinyusb_device pico_lwip pico_lwip_nosys pico_lwip_http pico_lwip_mdns pico_unique_id)
    pico_enable_stdio_uart(${target} 0)
    pico_enable_stdio_usb(${target} 0)
    pico_add_extra_outputs(${target})
endfunction()
