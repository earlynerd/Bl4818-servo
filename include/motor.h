/*
 * Motor Control State Machine
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

/* Motor states */
typedef enum {
    MOTOR_IDLE,         /* Outputs disabled, coasting */
    MOTOR_RUN,          /* Normal commutation */
    MOTOR_BRAKE,        /* Controlled deceleration / coast stop */
    MOTOR_FAULT,        /* Fault condition (overcurrent, stall) */
    MOTOR_REVERSING     /* Direction change in progress */
} motor_state_t;

/* Fault codes */
typedef enum {
    FAULT_NONE          = 0,
    FAULT_OVERCURRENT   = 1,
    FAULT_STALL         = 2,
    FAULT_HALL_INVALID  = 3,
    FAULT_WATCHDOG      = 4,
    FAULT_OVERVOLTAGE   = 5
} fault_code_t;

/* Initialize motor control */
void motor_init(void);

/* Set operating mode */
void motor_set_mode(uint8_t mode);
uint8_t motor_get_mode(void);

/* Set target values */
void motor_set_duty(int16_t duty);       /* Open-loop: -PWM_MAX to +PWM_MAX */
void motor_set_velocity(int16_t rpm);     /* Velocity mode target */
void motor_set_position(int32_t counts);  /* Position mode target */
void motor_set_torque_limit(uint16_t ma); /* Current limit */

/* Commands */
void motor_start(void);
void motor_stop(void);      /* Controlled brake / coast */
void motor_release(void);   /* Coast (disable outputs) */
void motor_clear_fault(void);

/* Fast-path polling for hall-driven commutation updates */
void motor_poll_fast(void);

/* State query */
motor_state_t motor_get_state(void);
fault_code_t motor_get_fault(void);
int16_t motor_get_velocity(void);    /* Current RPM */
int32_t motor_get_position(void);    /* Current encoder counts */
uint16_t motor_get_current(void);    /* Current in mA */

/* Called from main loop at CONTROL_LOOP_HZ rate */
void motor_update(void);

#endif /* MOTOR_H */
