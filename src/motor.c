/*
 * Motor Control State Machine
 *
 * Manages the overall motor state: idle, running, braking, faulted.
 * Runs the control loop (commutation, current limiting, PID) at 1kHz.
 *
 * Key improvements over stock firmware:
 * 1. Direction change WITHOUT stopping — uses active braking to decelerate,
 *    then immediately re-commutates in the new direction.
 * 2. Stall handling — detects stall via hall transition timeout, reduces
 *    current to prevent overheating, and maintains correct commutation.
 * 3. Current regulation — actively limits current via PWM duty reduction
 *    when the shunt ADC reading exceeds the soft limit.
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "motor.h"
#include "commutation.h"
#include "hall.h"
#include "encoder.h"
#include "pwm.h"
#include "adc.h"
#include "pid.h"

/* State */
static motor_state_t state;
static fault_code_t  fault;
static uint8_t       mode;
static int8_t        target_direction;  /* +1 or -1 */
static int16_t       target_duty;       /* Signed duty: + = forward */
static int16_t       target_velocity;   /* RPM */
static int32_t       target_position;   /* Encoder counts */
static uint16_t      torque_limit_ma;

/* Stall detection */
static uint16_t stall_counter;  /* ms since last hall transition */
static uint8_t  prev_hall_for_stall;

/* Current measurement */
static uint16_t current_ma;
static uint16_t velocity_rpm;

/* PID controllers — in XRAM to save IRAM */
static pid_t __xdata pid_velocity;
static pid_t __xdata pid_position;

/* Convert hall transition period to RPM */
static uint16_t period_to_rpm(uint16_t period)
{
    /*
     * Timer 2 runs at FSYS/12 = 2 MHz (0.5us per tick)
     * period = ticks between hall transitions
     * Electrical frequency = 1 / (period * 6 * 0.5us)  [6 transitions per e-rev]
     * Mechanical RPM = electrical_freq * 60 / MOTOR_POLE_PAIRS
     *
     * RPM = 2000000 * 60 / (period * 6 * MOTOR_POLE_PAIRS)
     *     = 120000000 / (period * 6 * 7)
     *     = 120000000 / (period * 42)
     *     ≈ 2857143 / period
     */
    if (period == 0 || period == 0xFFFF)
        return 0;

    uint32_t rpm = 2857143UL / (uint32_t)period;
    return (uint16_t)rpm;
}

void motor_init(void)
{
    state = MOTOR_IDLE;
    fault = FAULT_NONE;
    mode = MODE_OPEN_LOOP;
    target_direction = 1;
    target_duty = 0;
    target_velocity = 0;
    target_position = 0;
    torque_limit_ma = CURRENT_LIMIT_MA;
    stall_counter = 0;
    prev_hall_for_stall = 0;
    current_ma = 0;
    velocity_rpm = 0;

    pid_init(&pid_velocity, PID_VEL_KP, PID_VEL_KI, PID_VEL_KD,
             -(int16_t)PWM_MAX_DUTY, (int16_t)PWM_MAX_DUTY);
    pid_init(&pid_position, PID_POS_KP, PID_POS_KI, PID_POS_KD,
             -500, 500);  /* Position PID outputs velocity target */
}

void motor_set_mode(uint8_t m) { mode = m; }
uint8_t motor_get_mode(void) { return mode; }

void motor_set_duty(int16_t duty)
{
    target_duty = duty;
    if (duty >= 0)
        target_direction = 1;
    else
        target_direction = -1;
}

void motor_set_velocity(int16_t rpm)
{
    target_velocity = rpm;
    if (rpm >= 0)
        target_direction = 1;
    else
        target_direction = -1;
}

void motor_set_position(int32_t counts) { target_position = counts; }

void motor_set_torque_limit(uint16_t ma)
{
    if (ma > CURRENT_LIMIT_MA)
        ma = CURRENT_LIMIT_MA;
    torque_limit_ma = ma;
}

void motor_start(void)
{
    if (state == MOTOR_FAULT)
        return;  /* Must clear fault first */
    fault = FAULT_NONE;
    stall_counter = 0;
    state = MOTOR_RUN;
    pwm_enable();
}

void motor_stop(void)
{
    commutation_brake();
    state = MOTOR_BRAKE;
}

void motor_release(void)
{
    commutation_coast();
    pwm_disable();
    state = MOTOR_IDLE;
}

void motor_clear_fault(void)
{
    fault = FAULT_NONE;
    state = MOTOR_IDLE;
    commutation_coast();
    pwm_disable();
}

motor_state_t motor_get_state(void)  { return state; }
fault_code_t  motor_get_fault(void)  { return fault; }
int16_t  motor_get_velocity(void)    { return (int16_t)velocity_rpm; }
int32_t  motor_get_position(void)    { return encoder_get_position(); }
uint16_t motor_get_current(void)     { return current_ma; }

/*
 * Main control loop — called at CONTROL_LOOP_HZ (1 kHz)
 *
 * This is the heart of the firmware. It:
 * 1. Reads sensors (halls, current, encoder)
 * 2. Detects faults (overcurrent, stall)
 * 3. Runs PID loops (if in closed-loop mode)
 * 4. Applies commutation with current-limited duty cycle
 * 5. Handles direction changes seamlessly
 */
void motor_update(void)
{
    int16_t duty;
    int8_t actual_dir;

    /* ── Sensor Reading ──────────────────────────────────────────────── */

    /* Process hall sensor changes */
    hall_isr();  /* Poll-based: check for state changes */

    /* Read current */
    current_ma = adc_read_current_ma();

    /* Calculate velocity from hall period */
    velocity_rpm = period_to_rpm(hall_period());
    actual_dir = hall_direction();
    if (actual_dir < 0)
        velocity_rpm = -velocity_rpm;

    /* ── Fault Detection ─────────────────────────────────────────────── */

    /* Overcurrent — immediate shutdown */
    if (current_ma > CURRENT_LIMIT_MA) {
        pwm_fault_brake();
        state = MOTOR_FAULT;
        fault = FAULT_OVERCURRENT;
        return;
    }

    /* Hall sensor validity check */
    {
        uint8_t h = hall_read();
        if (h == 0 || h == 7) {
            pwm_fault_brake();
            state = MOTOR_FAULT;
            fault = FAULT_HALL_INVALID;
            return;
        }
    }

    /* Stall detection: if hall state hasn't changed for STALL_TIMEOUT_MS */
    if (state == MOTOR_RUN) {
        uint8_t h = hall_read();
        if (h == prev_hall_for_stall) {
            stall_counter++;
            if (stall_counter > STALL_TIMEOUT_MS && current_ma > CURRENT_WARN_MA) {
                /* Stall detected with high current — reduce duty to prevent damage.
                 * Don't fault out immediately; just limit current and keep trying.
                 * This is the key improvement for servo operation at stall. */
                stall_counter = STALL_TIMEOUT_MS;  /* Cap counter */
            }
        } else {
            prev_hall_for_stall = h;
            stall_counter = 0;
        }
    }

    /* ── State Machine ───────────────────────────────────────────────── */

    switch (state) {
    case MOTOR_IDLE:
        /* Nothing to do */
        return;

    case MOTOR_BRAKE:
        /* Active braking until speed is near zero */
        if (velocity_rpm == 0 || velocity_rpm > 60000) {
            /* Stopped or timer overflow — done braking */
            commutation_coast();
            state = MOTOR_IDLE;
        }
        return;

    case MOTOR_FAULT:
        /* Stay faulted until cleared */
        return;

    case MOTOR_REVERSING:
        /*
         * Direction change in progress.
         * We actively brake until the motor slows to a safe speed,
         * then switch commutation direction and resume.
         *
         * KEY IMPROVEMENT: Unlike stock firmware, we don't require
         * a complete stop. We brake down to ~10% speed, then
         * switch commutation. The back-EMF at low speed is small
         * enough that switching direction won't cause destructive
         * current spikes.
         */
        if (velocity_rpm < 200 || velocity_rpm > 60000) {
            /* Slow enough to reverse — switch to run in new direction */
            stall_counter = 0;
            state = MOTOR_RUN;
            pid_reset(&pid_velocity);
            pid_reset(&pid_position);
        } else {
            /* Still spinning — keep braking */
            commutation_brake();
            return;
        }
        break;  /* Fall through to RUN logic */

    case MOTOR_RUN:
        break;  /* Continue below */
    }

    /* ── Direction Change Detection ──────────────────────────────────── */

    /*
     * If the commanded direction differs from actual rotation direction
     * AND the motor is spinning above a threshold, initiate braking.
     * This allows seamless direction reversal for servo operation.
     */
    if (actual_dir != 0 && actual_dir != target_direction &&
        velocity_rpm > 200) {
        state = MOTOR_REVERSING;
        commutation_brake();
        return;
    }

    /* ── Control Loop ────────────────────────────────────────────────── */

    switch (mode) {
    case MODE_OPEN_LOOP:
        /* Direct duty cycle control */
        duty = target_duty;
        break;

    case MODE_VELOCITY:
        /* Velocity PID → duty cycle */
        duty = pid_update(&pid_velocity, target_velocity,
                          (int16_t)velocity_rpm);
        break;

    case MODE_POSITION:
        /* Position PID → velocity target → velocity PID → duty */
        {
            int32_t pos = encoder_get_position();
            int16_t vel_cmd;
            /* Clamp position error to int16 range */
            int32_t pos_err = target_position - pos;
            if (pos_err > 32000) pos_err = 32000;
            if (pos_err < -32000) pos_err = -32000;
            vel_cmd = pid_update(&pid_position, (int16_t)pos_err, 0);
            duty = pid_update(&pid_velocity, vel_cmd,
                              (int16_t)velocity_rpm);
        }
        break;

    case MODE_TORQUE:
        /* Current (torque) control — duty is adjusted to maintain target current */
        /* Simple proportional: reduce duty if current exceeds target */
        duty = target_duty;
        break;

    default:
        duty = 0;
        break;
    }

    /* ── Current Limiting ────────────────────────────────────────────── */

    /*
     * Soft current regulation: if measured current exceeds the warning
     * threshold, proportionally reduce the duty cycle. This prevents
     * overcurrent at stall while maintaining commutation.
     *
     * Also reduce duty during stall to prevent thermal damage.
     */
    if (current_ma > torque_limit_ma) {
        /* Hard limit: zero duty */
        duty = 0;
    } else if (current_ma > CURRENT_WARN_MA) {
        /* Proportional reduction */
        int32_t scale = (int32_t)(torque_limit_ma - current_ma) * 256 /
                        (torque_limit_ma - CURRENT_WARN_MA);
        if (scale < 0) scale = 0;
        duty = (int16_t)((int32_t)duty * scale / 256);
    }

    /* Stall current limiting */
    if (stall_counter >= STALL_TIMEOUT_MS) {
        /* At stall: limit duty to ~25% to maintain holding torque
         * without overheating */
        int16_t max_stall_duty = (int16_t)(PWM_MAX_DUTY / 4);
        if (duty > max_stall_duty) duty = max_stall_duty;
        if (duty < -max_stall_duty) duty = -max_stall_duty;
    }

    /* ── Apply Output ────────────────────────────────────────────────── */

    /* Determine direction from signed duty */
    int8_t dir;
    uint16_t abs_duty;
    if (duty >= 0) {
        dir = 1;
        abs_duty = (uint16_t)duty;
    } else {
        dir = -1;
        abs_duty = (uint16_t)(-duty);
    }

    target_direction = dir;
    pwm_set_duty(abs_duty);
    commutation_update(dir);
}
