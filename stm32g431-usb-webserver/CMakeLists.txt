cmake_minimum_required(VERSION 3.20)
project(stm32g431_usb_webserver C ASM)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
file(GLOB_RECURSE assets CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/web/*")
set(asset_header "${CMAKE_CURRENT_BINARY_DIR}/generated/web_assets.h")
add_custom_command(OUTPUT "${asset_header}"
    COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/../shared/rp2040-usb-web/tools/embed_web.py"
        "${CMAKE_CURRENT_SOURCE_DIR}/web" "${asset_header}"
    DEPENDS ${assets} "${CMAKE_CURRENT_SOURCE_DIR}/../shared/rp2040-usb-web/tools/embed_web.py")
set(CMAKE_C_STANDARD 11)
set(DEPS "${CMAKE_CURRENT_SOURCE_DIR}/deps")
set(TUSB "${DEPS}/tinyusb/src")
set(LWIP_DIR "${DEPS}/lwip")
foreach(dep tinyusb lwip cmsis cmsis-device-g4)
    if(NOT EXISTS "${DEPS}/${dep}/.git")
        message(FATAL_ERROR "Missing ${dep}; run Fetch-Dependencies.ps1 first")
    endif()
endforeach()
# Filelists.cmake rewrites files in the dependency tree; keep sources read-only.
file(GLOB lwipcore_SRCS CONFIGURE_DEPENDS ${LWIP_DIR}/src/core/*.c)
file(GLOB lwipcore4_SRCS CONFIGURE_DEPENDS ${LWIP_DIR}/src/core/ipv4/*.c)
add_executable(${PROJECT_NAME}
    src/main.c src/board.c src/system_control.c src/usb_descriptors.c src/http_page.c src/gpio_control.c ${asset_header}
    src/dhcpserver/dhcpserver.c src/dhcpserver/dhcp_options.c
    ${DEPS}/cmsis-device-g4/Source/Templates/system_stm32g4xx.c
    ${DEPS}/cmsis-device-g4/Source/Templates/gcc/startup_stm32g431xx.s
    ${TUSB}/tusb.c ${TUSB}/common/tusb_fifo.c
    ${TUSB}/device/usbd.c
    ${TUSB}/class/net/ncm_device.c
    ${TUSB}/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    ${TUSB}/portable/st/stm32_fsdev/fsdev_common.c
    ${lwipcore_SRCS} ${lwipcore4_SRCS}
    ${LWIP_DIR}/src/netif/ethernet.c
    ${LWIP_DIR}/src/apps/http/httpd.c ${LWIP_DIR}/src/apps/http/fs.c)
target_include_directories(${PROJECT_NAME} PRIVATE
    src "${CMAKE_CURRENT_BINARY_DIR}/generated" ${TUSB} ${LWIP_DIR}/src/include
    ${DEPS}/cmsis/CMSIS/Core/Include ${DEPS}/cmsis-device-g4/Include)
target_compile_definitions(${PROJECT_NAME} PRIVATE STM32G431xx CFG_TUSB_MCU=OPT_MCU_STM32G4)
target_compile_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -Os -g3
    -ffunction-sections -fdata-sections -fno-common -fstack-usage
    -Wall -Wextra)
target_link_options(${PROJECT_NAME} PRIVATE
    -mcpu=cortex-m4 -mthumb -mfloat-abi=soft -nostartfiles
    --specs=nano.specs --specs=nosys.specs
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g431cb.ld
    -Wl,--wrap=SystemInit,--gc-sections,--print-memory-usage,-Map=${PROJECT_NAME}.map)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/stm32g431cb.ld)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX .elf)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
