#include "sdk.h"
#include <assert.h>
struct fake_pin fake_pins[30];
struct fake_pwm fake_pwms[8];
struct fake_event fake_events[FAKE_EVENT_CAPACITY];
unsigned fake_event_count;
void fake_clear_events(void) { fake_event_count=0; }
static void record(enum fake_event_kind kind, unsigned target, unsigned a, unsigned b, unsigned c) {
    assert(fake_event_count<FAKE_EVENT_CAPACITY);
    fake_events[fake_event_count++]=(struct fake_event){kind,target,a,b,c};
}
void gpio_init(unsigned p) { record(EVENT_GPIO_INIT,p,0,0,0); fake_pins[p] = (struct fake_pin){.drive=GPIO_DRIVE_STRENGTH_4MA,.hysteresis=true}; }
void gpio_set_dir(unsigned p, bool v) { record(EVENT_DIR,p,v,0,0); fake_pins[p].output=v; }
void gpio_put(unsigned p, bool v) { record(EVENT_LATCH,p,v,0,0); fake_pins[p].latch=v; }
void gpio_set_function(unsigned p, unsigned v) { record(EVENT_FUNCTION,p,v,0,0); fake_pins[p].function=v; }
void gpio_set_input_enabled(unsigned p, bool v) { record(EVENT_INPUT,p,v,0,0); fake_pins[p].input=v; }
void gpio_disable_pulls(unsigned p) { gpio_set_pulls(p,false,false); }
void gpio_set_pulls(unsigned p, bool up, bool down) { record(EVENT_PULLS,p,up,down,0); fake_pins[p].up=up; fake_pins[p].down=down; }
bool gpio_get_out_level(unsigned p) { return fake_pins[p].latch; }
bool gpio_get(unsigned p) { return fake_pins[p].output ? fake_pins[p].latch : fake_pins[p].up; }
bool gpio_is_dir_out(unsigned p) { return fake_pins[p].output; }
bool gpio_is_pulled_up(unsigned p) { return fake_pins[p].up; }
bool gpio_is_pulled_down(unsigned p) { return fake_pins[p].down; }
void gpio_set_drive_strength(unsigned p, enum gpio_drive_strength v) { record(EVENT_DRIVE,p,v,0,0); fake_pins[p].drive=v; }
enum gpio_drive_strength gpio_get_drive_strength(unsigned p) { return fake_pins[p].drive; }
void gpio_set_slew_rate(unsigned p, enum gpio_slew_rate v) { record(EVENT_SLEW,p,v,0,0); fake_pins[p].slew=v; }
enum gpio_slew_rate gpio_get_slew_rate(unsigned p) { return fake_pins[p].slew; }
void gpio_set_input_hysteresis_enabled(unsigned p, bool v) { record(EVENT_HYSTERESIS,p,v,0,0); fake_pins[p].hysteresis=v; }
bool gpio_is_input_hysteresis_enabled(unsigned p) { return fake_pins[p].hysteresis; }
unsigned pwm_gpio_to_slice_num(unsigned p) { return (p%16)/2; }
unsigned pwm_gpio_to_channel(unsigned p) { return p%2; }
void pwm_set_chan_level(unsigned s, unsigned c, unsigned v) { record(EVENT_PWM_LEVEL,s,c,v,0); fake_pwms[s].level[c]=v; }
void pwm_set_enabled(unsigned s, bool v) { record(EVENT_PWM_ENABLED,s,v,0,0); fake_pwms[s].enabled=v; }
pwm_config pwm_get_default_config(void) { return (pwm_config){0}; }
void pwm_config_set_clkdiv_int_frac(pwm_config *c, unsigned d, unsigned f) { c->div=d; c->frac=f; }
void pwm_config_set_wrap(pwm_config *c, unsigned w) { c->wrap=w; }
void pwm_init(unsigned s, const pwm_config *c, bool v) { record(EVENT_PWM_INIT,s,c->div,c->frac,c->wrap); fake_pwms[s].config=*c; fake_pwms[s].enabled=v; }
void pwm_set_both_levels(unsigned s, unsigned a, unsigned b) { record(EVENT_PWM_BOTH,s,a,b,0); fake_pwms[s].level[0]=a; fake_pwms[s].level[1]=b; }
unsigned clock_get_hz(unsigned clock) { (void)clock; return 125000000; }
void adc_init(void) { record(EVENT_ADC_INIT,0,0,0,0);}
void adc_gpio_init(unsigned p) { record(EVENT_ADC_GPIO,p,0,0,0); gpio_set_input_enabled(p,false); gpio_disable_pulls(p); }
void adc_select_input(unsigned input) { assert(input<4); }
unsigned adc_read(void) { return 2048; }
