#include "gpio_control.h"
#include "stm32g4xx.h"
/* Only pins bonded in UFQFPN48; same order as web/stm32-pins.js. */
gpio_state_t gpio_states[GPIO_COUNT];
static const uint8_t extras[] = {4,6,13,14,15,0,1,10,11};
static GPIO_TypeDef *port(unsigned p) { return p<16 ? GPIOA : p<32 ? GPIOB : p==37 || p==38 ? GPIOF : GPIOC; }
static unsigned bit(unsigned p) { return p<32 ? p%16 : extras[p-32]; }
bool gpio_available(unsigned p) { return (p<35 || p==39 || p==40) && p!=11 && p!=12 && p!=13 && p!=14 && p!=24; }
bool gpio_level(unsigned p) { return p<GPIO_COUNT && (port(p)->IDR & (1U<<bit(p))); }
static TIM_TypeDef *timer(unsigned p, unsigned *channel, unsigned *af) {
    *af=2;
    if(p<4) { *channel=p; *af=1; return TIM2; }
    if(p==6 || p==7) { *channel=p-6; return TIM3; }
    if(p==16 || p==17) { *channel=p-14; return TIM3; }
    if(p==22 || p==23 || p==25) { *channel=p-22; return TIM4; }
    return NULL;
}
int gpio_apply(unsigned p, gpio_state_t s) {
    if(p>=GPIO_COUNT) return 400;
    if(!gpio_available(p)) return 403;
    if(s.mode>3 || s.pull>2 || s.value>1 || s.fast>1) return 400;
    unsigned ch,af; TIM_TypeDef *tim=timer(p,&ch,&af);
    if(s.mode==3) {
        if(!tim || s.frequency<10 || s.frequency>100000 || s.duty>100) return 400;
        for(unsigned i=0;i<GPIO_COUNT;++i) {
            unsigned other,other_af;
            if(i!=p && gpio_states[i].mode==3 && timer(i,&other,&other_af)==tim && gpio_states[i].frequency!=s.frequency) return 409;
        }
    }
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN | RCC_AHB2ENR_GPIOFEN;
    (void)RCC->AHB2ENR;
    if(tim && gpio_states[p].mode==3) tim->CCER &= ~(1U<<(ch*4));
    GPIO_TypeDef *g=port(p); unsigned n=bit(p), shift=n*2;
    g->MODER &= ~(3U<<shift); /* disconnect output before reconfiguration */
    g->BSRR = 1U<<(n+(s.value ? 0 : 16));
    g->OTYPER = (g->OTYPER & ~(1U<<n)) | ((s.mode==2 ? 1U:0U)<<n);
    g->OSPEEDR = (g->OSPEEDR & ~(3U<<shift)) | ((s.fast?3U:0U)<<shift);
    g->PUPDR = (g->PUPDR & ~(3U<<shift)) | ((s.mode==0?s.pull:0U)<<shift);
    if(s.mode==3) {
        RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN | RCC_APB1ENR1_TIM3EN | RCC_APB1ENR1_TIM4EN;
        (void)RCC->APB1ENR1;
        /* All timers run at HSI16 (APB1 prescaler = 1). */
        uint32_t divider=(16000000U/s.frequency+65535U)/65536U;
        uint32_t period=16000000U/(divider*s.frequency);
        bool occupied=false;
        for(unsigned i=0;i<GPIO_COUNT;++i) {
            unsigned c,a;
            if(i!=p && gpio_states[i].mode==3 && timer(i,&c,&a)==tim) occupied=true;
        }
        if(!occupied) { tim->CR1=0; tim->PSC=divider-1; tim->ARR=period-1; tim->CNT=0; }
        volatile uint32_t *ccmr=ch<2 ? &tim->CCMR1 : &tim->CCMR2;
        unsigned cs=(ch%2)*8;
        *ccmr=(*ccmr & ~(255U<<cs)) | (0x68U<<cs);
        (&tim->CCR1)[ch]=(period*s.duty)/100;
        if(!occupied) tim->EGR=TIM_EGR_UG;
        tim->CCER |= 1U<<(ch*4);
        tim->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
        g->AFR[n/8]=(g->AFR[n/8]&~(15U<<((n%8)*4))) | (af<<((n%8)*4));
    }
    g->MODER |= (s.mode==0?0U:s.mode==3?2U:1U)<<shift;
    gpio_states[p]=s;
    return 200;
}
