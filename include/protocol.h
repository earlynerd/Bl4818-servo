/*
 * Production serial protocol: binary ring protocol only.
 *
 * The separate bench image retains an ASCII debug protocol in bench_main.c.
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

void protocol_init(void);
void protocol_poll(void);

#endif /* PROTOCOL_H */
