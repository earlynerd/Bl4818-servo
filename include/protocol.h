/*
 * Serial Command Protocol — ASCII text commands over UART
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_BUF_SIZE  48

/* Initialize protocol parser */
void protocol_init(void);

/* Process incoming bytes — call from main loop */
void protocol_poll(void);

/* Send status response */
void protocol_send_status(void);

#endif /* PROTOCOL_H */
