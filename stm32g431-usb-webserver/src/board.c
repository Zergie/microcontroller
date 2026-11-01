#include "stm32g4xx.h"
#include "tusb.h"
#include "board.h"

static volatile uint32_t ticks;
/* NOLOAD SRAM survives NVIC_SystemReset, but is never treated as initialized. */
static volatile uint32_t dfu_request[2] __attribute__((section(".noinit")));
#define DFU_MAGIC 0x44465531UL
#define SYSTEM_ROM 0x1fff0000UL /* AN2606: STM32G431xx/441xx */

__attribute__((naked,noreturn)) static void jump_to_rom(void) {
    __asm volatile(
        "ldr r0, =0x1fff0000\n"
        "ldr r1, [r0]\n"
        "ldr r2, [r0, #4]\n"
        "movs r3, #0\n"
        "msr control, r3\n"
        "msr basepri, r3\n"
        "msr faultmask, r3\n"
        "msr msp, r1\n"
        "dsb\n"
        "isb\n"
        "cpsie i\n"
        "bx r2\n");
}

void __real_SystemInit(void);
/* Startup calls this before data/BSS initialization and peripheral setup. */
void __wrap_SystemInit(void) {
    __real_SystemInit();
    bool requested=(RCC->CSR & RCC_CSR_SFTRSTF) &&
        dfu_request[0]==DFU_MAGIC && dfu_request[1]==(uint32_t)~DFU_MAGIC;
    dfu_request[0]=0; dfu_request[1]=0;
    RCC->CSR |= RCC_CSR_RMVF;
    if (!requested) return;
    __disable_irq();
    SysTick->CTRL=0; SysTick->LOAD=0; SysTick->VAL=0;
    for(unsigned i=0;i<8;++i) {NVIC->ICER[i]=0xffffffffU; NVIC->ICPR[i]=0xffffffffU;}
    SCB->ICSR=SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    SYSCFG->MEMRMP=(SYSCFG->MEMRMP & ~SYSCFG_MEMRMP_MEM_MODE) | SYSCFG_MEMRMP_MEM_MODE_0;
    SCB->VTOR=SYSTEM_ROM;
    __DSB(); __ISB();
    jump_to_rom();
}

void board_reset_to_dfu(void) {
    tud_disconnect();
    /* Give the USB host a visible disconnect before ROM DFU enumerates. */
    uint32_t disconnected_at=sys_now();
    while((uint32_t)(sys_now()-disconnected_at)<250U) {}
    __disable_irq();
    dfu_request[0]=DFU_MAGIC; dfu_request[1]=(uint32_t)~DFU_MAGIC;
    __DSB();
    NVIC_SystemReset();
}

void board_init(void) {
    SCB->VTOR = FLASH_BASE;
    /* Keep the reset HSI16 system clock: no external crystal required. */
    SystemCoreClockUpdate();
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
    RCC->APB1ENR1 |= RCC_APB1ENR1_USBEN | RCC_APB1ENR1_CRSEN;
    (void)RCC->APB1ENR1;
    RCC->CRRCR |= RCC_CRRCR_HSI48ON;
    while (!(RCC->CRRCR & RCC_CRRCR_HSI48RDY)) {}
    RCC->CCIPR &= ~RCC_CCIPR_CLK48SEL; /* HSI48 */
    /* CRS: USB SOF at 1 kHz trims the 48 MHz USB oscillator. */
    CRS->CFGR = (47999U << CRS_CFGR_RELOAD_Pos) |
                (34U << CRS_CFGR_FELIM_Pos) | CRS_CFGR_SYNCSRC_1;
    CRS->CR = (32U << CRS_CR_TRIM_Pos) | CRS_CR_AUTOTRIMEN | CRS_CR_CEN;
    /* USB owns PA11/PA12. FSDEV needs no alternate-function GPIO setup. */
    GPIOA->MODER &= ~((3U << 22) | (3U << 24));
    GPIOA->PUPDR &= ~((3U << 22) | (3U << 24));
    NVIC_SetPriority(USB_HP_IRQn, 2);
    NVIC_SetPriority(USB_LP_IRQn, 2);
    if (SysTick_Config(SystemCoreClock / 1000U)) platform_panic();
}

void SysTick_Handler(void) { ++ticks; }
uint32_t sys_now(void) { return ticks; }
uint32_t tusb_time_millis_api(void) { return ticks; }
void USB_HP_IRQHandler(void) { tud_int_handler(0); }
void USB_LP_IRQHandler(void) { tud_int_handler(0); }
void platform_panic(void) { __disable_irq(); for (;;) {} }
void HardFault_Handler(void) { platform_panic(); }
void _init(void) {}
