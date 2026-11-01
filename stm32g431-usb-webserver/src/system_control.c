#include <stdbool.h>
#include <stdint.h>
#include "board.h"
#include "system_control.h"

static bool pending;
static uint32_t requested_at;
void system_request_dfu(void) {
    /* Retries must not postpone the transition indefinitely. */
    if (!pending) { requested_at=sys_now(); pending=true; }
}
void system_control_poll(void) {
    if (pending && (uint32_t)(sys_now()-requested_at)>=1500U) {
        pending=false;
        board_reset_to_dfu();
    }
}
