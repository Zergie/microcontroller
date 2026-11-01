#include <stdint.h>
#include <stddef.h>
typedef struct {uint32_t MODER,OTYPER,OSPEEDR,PUPDR,IDR,ODR,BSRR,AFR[2];} GPIO_TypeDef;
typedef struct {uint32_t CR1,PSC,ARR,CNT,CCER,CCMR1,CCMR2,CCR1,CCR2,CCR3,CCR4,EGR;} TIM_TypeDef;
typedef struct {uint32_t AHB2ENR,APB1ENR1;} RCC_TypeDef;
extern GPIO_TypeDef fake_gpio[4];
extern TIM_TypeDef fake_timer[3];
extern RCC_TypeDef fake_rcc;
#define GPIOA (&fake_gpio[0])
#define GPIOB (&fake_gpio[1])
#define GPIOC (&fake_gpio[2])
#define GPIOF (&fake_gpio[3])
#define TIM2 (&fake_timer[0])
#define TIM3 (&fake_timer[1])
#define TIM4 (&fake_timer[2])
#define RCC (&fake_rcc)
#define RCC_AHB2ENR_GPIOAEN 1
#define RCC_AHB2ENR_GPIOBEN 2
#define RCC_AHB2ENR_GPIOCEN 4
#define RCC_AHB2ENR_GPIOFEN 32
#define RCC_APB1ENR1_TIM2EN 1
#define RCC_APB1ENR1_TIM3EN 2
#define RCC_APB1ENR1_TIM4EN 4
#define TIM_EGR_UG 1
#define TIM_CR1_CEN 1
#define TIM_CR1_ARPE 128
extern const uint8_t test_uid[12];
#define UID_BASE test_uid
