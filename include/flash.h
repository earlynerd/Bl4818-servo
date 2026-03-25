/*
 * Flash IAP — Non-volatile parameter storage
 */
#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>

/* Parameters stored in flash */
typedef struct {
    uint8_t  magic;         /* 0xA5 = valid */
    uint8_t  mode;          /* Operating mode */
    uint16_t torque_limit;  /* Current limit in mA */
    int16_t  pid_pos_kp;
    int16_t  pid_pos_ki;
    int16_t  pid_pos_kd;
    int16_t  pid_vel_kp;
    int16_t  pid_vel_ki;
    int16_t  pid_vel_kd;
    uint16_t encoder_cpr;   /* Encoder counts per revolution */
    uint8_t  checksum;      /* Simple checksum */
} params_t;

#define PARAMS_MAGIC    0xA5

/* Load parameters from flash (returns defaults if invalid) */
void flash_load_params(params_t *p);

/* Save parameters to flash */
void flash_save_params(const params_t *p);

/* Reset parameters to factory defaults */
void flash_default_params(params_t *p);

#endif /* FLASH_H */
