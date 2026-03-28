/*
 * Serial Command Protocol — ASCII text commands over UART
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_BUF_SIZE  48

void protocol_init(void);
void protocol_poll(void);

#endif /* PROTOCOL_H */
