#ifndef TEST1_ROUTE_CONTROLLER_H
#define TEST1_ROUTE_CONTROLLER_H

#include <stdint.h>

void route_controller_run(void);

/* Non-zero once the RK arm link has reported ready.  Exposed as a getter so
 * the underlying flag can stay file-static inside route_controller.c. */
uint8_t route_controller_rk_link_ready(void);

#endif
