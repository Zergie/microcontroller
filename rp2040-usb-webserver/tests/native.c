#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "dhcpserver/dhcp_options.h"
#include "gpio_api.h"
#include "sdk.h"

static void dhcp_tests(void) {
    struct dhcp_request_options result;
    const uint8_t valid[] = {99,130,83,99,0,53,1,3,50,4,192,168,7,16,255};
    assert(dhcp_parse_options(valid,sizeof(valid),&result));
    assert(result.message_type==3 && result.has_requested_ip && result.requested_ip[3]==16);
    for (size_t n=0;n<sizeof(valid);++n) assert(!dhcp_parse_options(valid,n,&result));
    uint8_t packet[312] = {99,130,83,99};
    // Original overflow: two option lengths lead to the very last byte.
    packet[4]=1; packet[5]=253; packet[259]=1; packet[260]=50; packet[311]=1;
    assert(!dhcp_parse_options(packet,sizeof(packet),&result));
    const uint8_t duplicate[] = {99,130,83,99,53,1,1,53,1,3,255};
    const uint8_t short_ip[] = {99,130,83,99,53,1,3,50,1,16,255};
    const uint8_t short_type[] = {99,130,83,99,53,0,255};
    assert(!dhcp_parse_options(duplicate,sizeof(duplicate),&result));
    assert(!dhcp_parse_options(short_ip,sizeof(short_ip),&result));
    assert(!dhcp_parse_options(short_type,sizeof(short_type),&result));
    // Deterministic malformed payloads, including short headers and bad cookies.
    uint32_t seed=12345;
    for (unsigned run=0;run<100000;++run) {
        size_t n=run%sizeof(packet);
        for (size_t i=0;i<n;++i) { seed=1664525*seed+1013904223; packet[i]=seed>>24; }
        if (n>=4 && run%2) memcpy(packet,valid,4);
        dhcp_parse_options(packet,n,&result);
    }
    puts("DHCP: bounds regression, PAD/END, truncation, lengths, duplicates, 100000 fuzz cases OK");
}

static int command(const char *path) { bool led; return gpio_api_command(path,&led); }

static void rejected(const char *path, int expected) {
    struct fake_pin before[30]; struct fake_pwm pwm_before[8];
    memcpy(before,fake_pins,sizeof(before)); memcpy(pwm_before,fake_pwms,sizeof(pwm_before));
    fake_clear_events();
    assert(command(path)==expected);
    assert(fake_event_count==0);
    assert(memcmp(before,fake_pins,sizeof(before))==0);
    assert(memcmp(pwm_before,fake_pwms,sizeof(pwm_before))==0);
}

static void gpio_tests(void) {
    for (unsigned i=0;i<30;++i) gpio_init(i);
    gpio_control_init(); gpio_control_boot_ready();
    assert(gpio_control_led_on());
    rejected("/api/gpio/42949672960/on",400);
    rejected("/api/gpio/-1/on",400);
    rejected("/api/gpio/30/on",400);
    rejected("/api/gpio/23/on",403);
    rejected("/api/gpio/25/onward",404);
    rejected("/api/gpio/25/on/drive/3/slow",400);
    rejected("/api/gpio/25/input/drive/4/slow",400);
    rejected("/api/gpio/25/on/hysteresis/off",400);
    rejected("/api/gpio/25/on/drive/4/fast/hysteresis/on",400);
    rejected("/api/gpio/25/pwm/1000/50junk",400);
    rejected("/api/gpio/25/pwm/9/50/drive/4/slow",400);
    rejected("/api/gpio/25/analog",400);
    assert(command("/api/gpio/25/pwm/1000/50/drive/8/fast")==200);
    rejected("/api/gpio/9/pwm/1000/50",409);
    rejected("/api/gpio/8/pwm/2000/50/drive/12/slow",409);
    assert(command("/api/gpio/8/pwm/1000/25")==200);
    unsigned other=fake_pwms[4].level[1];
    assert(command("/api/gpio/8/input")==200);
    assert(fake_pwms[4].enabled && fake_pwms[4].level[1]==other);
    assert(command("/api/gpio/25/open-drain/release/drive/2/slow")==200);
    assert(!fake_pins[25].output && !fake_pins[25].latch && fake_pins[25].drive==GPIO_DRIVE_STRENGTH_2MA);
    assert(command("/api/gpio/25/open-drain/low")==200);
    assert(fake_pins[25].output && !fake_pins[25].latch && !fake_pwms[4].enabled);
    assert(command("/api/gpio/25/input-buskeeper/hysteresis/off")==200);
    assert(fake_pins[25].up && fake_pins[25].down && !fake_pins[25].hysteresis);
    assert(command("/api/gpio/26/analog")==200);
    assert(!fake_pins[26].input);
    struct gpio_snapshot pin;
    gpio_control_read(26,&pin);
    assert(pin.adc_raw==2048 && pin.millivolts==1650);
    assert(command("/api/gpio/26/input")==200 && fake_pins[26].input);
    assert(command("/api/led/on")==200 && gpio_control_led_on());
    char json[12288];
    assert(gpio_api_snapshot(json,sizeof(json)) && strstr(json,"\"pin\":29"));
    assert(!gpio_api_snapshot(json,10));
    puts("GPIO/API: parsing, validation before writes, PWM conflicts/sharing, modes, pads, JSON bounds OK");
}

static int event_index(enum fake_event_kind kind, unsigned target, unsigned value) {
    for (unsigned i=0; i<fake_event_count; ++i) {
        const struct fake_event *event=&fake_events[i];
        if (event->kind==kind && event->target==target && event->a==value) return (int)i;
    }
    return -1;
}

static void transition_tests(void) {
    const char *modes[]={"input", "input-pullup", "input-pulldown", "input-buskeeper",
        "off", "on", "pwm/1000/37", "analog", "open-drain/low", "open-drain/release"};
    for (unsigned source=0; source<10; ++source) {
        for (unsigned target=0; target<10; ++target) {
            char path[96];
            fake_clear_events();
            gpio_init(26);
            gpio_control_init();
            snprintf(path,sizeof(path),"/api/gpio/26/%s",modes[source]);
            assert(command(path)==200);
            fake_clear_events();
            const char *pad=target<4 ? "/hysteresis/off" :
                target==7 ? "" : "/drive/12/fast";
            snprintf(path,sizeof(path),"/api/gpio/26/%s%s",modes[target],pad);
            assert(command(path)==200);
            unsigned sio_count=0, latch_count=0;
            for (unsigned i=0; i<fake_event_count; ++i) {
                if (fake_events[i].kind==EVENT_FUNCTION && fake_events[i].target==26 &&
                    fake_events[i].a==GPIO_FUNC_SIO) ++sio_count;
                if (fake_events[i].kind==EVENT_LATCH && fake_events[i].target==26) ++latch_count;
            }
            assert(sio_count==1 && latch_count==1);
            int release=event_index(EVENT_DIR,26,false);
            int mux=event_index(EVENT_FUNCTION,26,GPIO_FUNC_SIO);
            int input=event_index(EVENT_INPUT,26,true);
            assert(release>=0 && mux>release && input>mux);
            if (source==6) {
                int zero=event_index(EVENT_PWM_LEVEL,5,0);
                assert(zero>=0 && zero<release && fake_events[zero].b==0);
                int disable=event_index(EVENT_PWM_ENABLED,5,false);
                assert(disable>=0 && disable<release);
            }
            if (target==4 || target==5 || target==8) {
                int enable=event_index(EVENT_DIR,26,true);
                int latch=event_index(EVENT_LATCH,26,target==5);
                assert(latch>=0 && latch<enable && enable>mux);
                int drive=event_index(EVENT_DRIVE,26,GPIO_DRIVE_STRENGTH_12MA);
                int slew=event_index(EVENT_SLEW,26,GPIO_SLEW_RATE_FAST);
                assert(drive>=0 && drive<enable && slew>=0 && slew<enable);
            }
            if (target==8 || target==9) {
                for (unsigned i=0; i<fake_event_count; ++i)
                    if (fake_events[i].kind==EVENT_LATCH) assert(fake_events[i].a==false);
                assert(fake_pins[26].output==(target==8) && !fake_pins[26].latch);
            }
            if (target==6) {
                int activate=event_index(EVENT_FUNCTION,26,GPIO_FUNC_PWM);
                int drive=event_index(EVENT_DRIVE,26,GPIO_DRIVE_STRENGTH_12MA);
                int slew=event_index(EVENT_SLEW,26,GPIO_SLEW_RATE_FAST);
                int init=event_index(EVENT_PWM_INIT,5,fake_pwms[5].config.div);
                assert(activate>input && drive>=0 && drive<activate && slew>=0 && slew<activate);
                assert(init>=0 && init<activate);
                assert(event_index(EVENT_PWM_ENABLED,5,true)>activate);
            } else {
                struct gpio_snapshot state;
                gpio_control_read(26,&state);
                assert(state.frequency==0 && state.actual_frequency==0 && state.duty==0);
            }
            if (target<4) assert(event_index(EVENT_HYSTERESIS,26,false)>input);
            if (target==7) assert(event_index(EVENT_ADC_GPIO,26,0)>input && !fake_pins[26].input);
            else assert(fake_pins[26].input);
            struct gpio_snapshot final;
            gpio_control_read(26,&final);
            enum pin_mode expected_mode=target<4 ? PIN_INPUT : target<6 ? PIN_OUTPUT :
                target==6 ? PIN_PWM : target==7 ? PIN_ANALOG : PIN_OPEN_DRAIN;
            enum pin_pull expected_pull=target==1 ? PULL_UP : target==2 ? PULL_DOWN :
                target==3 ? PULL_KEEPER : PULL_NONE;
            bool driven=target==4 || target==5 || target==8;
            bool output_pad=target>=4 && target!=7;
            assert(final.mode==expected_mode && final.pull==expected_pull);
            assert(final.output_enabled==driven && final.output_latch==(target==5));
            assert(final.drive_ma==(output_pad ? 12u : 4u));
            assert(final.fast_slew==output_pad && final.hysteresis==(target>=4));
            if (target==6) assert(final.frequency==1000 && final.duty==37);
        }
    }
    fake_clear_events();
    gpio_control_init();
    assert(command("/api/gpio/26/pwm/1000/25")==200);
    assert(command("/api/gpio/27/pwm/1000/75")==200);
    unsigned sibling=fake_pwms[5].level[1];
    fake_clear_events();
    assert(command("/api/gpio/26/input")==200);
    assert(fake_pwms[5].enabled && fake_pwms[5].level[1]==sibling);
    assert(event_index(EVENT_PWM_ENABLED,5,false)==-1);
    assert(command("/api/gpio/27/input")==200);
    puts("GPIO transitions: 100 mode pairs, release/mux/latch/pad/activation order, ADC buffer and PWM sibling OK");
}

static void snapshot_contract(bool emit) {
    const char *commands[] = {
        "/api/gpio/0/input/hysteresis/off", "/api/gpio/1/input-pullup",
        "/api/gpio/2/input-pulldown", "/api/gpio/3/input-buskeeper",
        "/api/gpio/4/on/drive/12/fast", "/api/gpio/5/off/drive/2/slow",
        "/api/gpio/6/pwm/2345/37/drive/8/fast",
        "/api/gpio/7/open-drain/low", "/api/gpio/8/open-drain/release",
        "/api/gpio/26/analog"
    };
    for (unsigned pin=0; pin<30; ++pin) gpio_init(pin);
    gpio_control_init();
    for (unsigned i=0; i<sizeof(commands)/sizeof(commands[0]); ++i) assert(command(commands[i])==200);
    char json[12288];
    assert(gpio_api_snapshot(json,sizeof(json)));
    size_t length=strlen(json);
    unsigned char guarded[12290];
    for (size_t capacity=0; capacity<=length+1; ++capacity) {
        memset(guarded,0xa5,sizeof(guarded));
        bool ok=gpio_api_snapshot((char *)guarded+1,capacity);
        assert(ok==(capacity==length+1));
        assert(guarded[0]==0xa5 && guarded[capacity+1]==0xa5);
        if (ok) assert(memcmp(guarded+1,json,length+1)==0);
    }
    if (emit) puts(json);
    else puts("GPIO JSON: all modes, exact fit/NUL, every short capacity and canaries OK");
}

int main(int argc, char **argv) {
    if (argc==2 && strcmp(argv[1],"--snapshot-contract")==0) { snapshot_contract(true); return 0; }
    dhcp_tests(); gpio_tests(); transition_tests(); fake_clear_events(); snapshot_contract(false);
}
