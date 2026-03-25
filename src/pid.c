/*
 * PID Controller — Fixed-point (Q8) implementation
 *
 * All gains are in Q8 fixed-point: actual_gain = value / 256.
 * This avoids floating-point math on the 8051.
 */

#include "pid.h"
#include "ms51_config.h"

void pid_init(pid_t *pid, int16_t kp, int16_t ki, int16_t kd,
              int16_t out_min, int16_t out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0;
    pid->prev_error = 0;
    pid->out_min = out_min;
    pid->out_max = out_max;
    /* Anti-windup: limit integral to prevent output saturation */
    pid->int_max = (int32_t)out_max * PID_SCALE * 2;
}

int16_t pid_update(pid_t *pid, int16_t setpoint, int16_t measurement)
{
    int16_t error = setpoint - measurement;
    int32_t output;

    /* Proportional term */
    int32_t p_term = (int32_t)pid->kp * error;

    /* Integral term with anti-windup */
    pid->integral += (int32_t)pid->ki * error;
    if (pid->integral > pid->int_max)
        pid->integral = pid->int_max;
    else if (pid->integral < -pid->int_max)
        pid->integral = -pid->int_max;

    /* Derivative term (on error, not measurement, for simplicity) */
    int32_t d_term = (int32_t)pid->kd * (error - pid->prev_error);
    pid->prev_error = error;

    /* Sum and scale back from Q8 */
    output = (p_term + pid->integral + d_term) / PID_SCALE;

    /* Clamp output */
    if (output > pid->out_max)
        output = pid->out_max;
    else if (output < pid->out_min)
        output = pid->out_min;

    return (int16_t)output;
}

void pid_reset(pid_t *pid)
{
    pid->integral = 0;
    pid->prev_error = 0;
}
