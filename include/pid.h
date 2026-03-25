/*
 * PID Controller — Fixed-point implementation
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    int16_t kp;         /* Proportional gain (Q8 fixed-point) */
    int16_t ki;         /* Integral gain */
    int16_t kd;         /* Derivative gain */
    int32_t integral;   /* Accumulated integral */
    int16_t prev_error; /* Previous error for derivative */
    int16_t out_min;    /* Output clamp minimum */
    int16_t out_max;    /* Output clamp maximum */
    int32_t int_max;    /* Anti-windup integral limit */
} pid_t;

/* Initialize PID controller with gains and output limits */
void pid_init(pid_t *pid, int16_t kp, int16_t ki, int16_t kd,
              int16_t out_min, int16_t out_max);

/* Compute PID output given setpoint and measurement */
int16_t pid_update(pid_t *pid, int16_t setpoint, int16_t measurement);

/* Reset integral and derivative state */
void pid_reset(pid_t *pid);

#endif /* PID_H */
