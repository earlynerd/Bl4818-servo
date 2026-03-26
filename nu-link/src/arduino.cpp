#include <Arduino.h>
#define _DEBUG
#include "n51_pgm.h"
/* Lolin(WeMOS) D1 mini */
/* 80MHz: 1 cycle = 12.5ns */
#ifndef ARDUINO_AVR_MEGA2560
#define DAT   11
#define CLK   12
#define RST   13
#define PWR   10
#else
#define DAT   52
#define CLK   50
#define RST   48
#define PWR   46
#endif

#ifndef TARGET_POWER_ACTIVE_HIGH
#define TARGET_POWER_ACTIVE_HIGH 1
#endif

#ifndef TARGET_POWER_DEFAULT_ON
#define TARGET_POWER_DEFAULT_ON 1
#endif
extern Adafruit_USBD_CDC DebugSerial;
extern "C" {

static uint8_t initialized = 0;
static uint8_t power_pin_initialized = 0;

void N51PGM_release_pins(void);

static void N51PGM_init_power_pin(void)
{
  if (power_pin_initialized) {
    return;
  }

  pinMode(PWR, OUTPUT);
#if TARGET_POWER_ACTIVE_HIGH
  digitalWrite(PWR, TARGET_POWER_DEFAULT_ON ? HIGH : LOW);
#else
  digitalWrite(PWR, TARGET_POWER_DEFAULT_ON ? LOW : HIGH);
#endif
  power_pin_initialized = 1;
}

int N51PGM_init(void)
{
  N51PGM_init_power_pin();
  pinMode(CLK, OUTPUT);
  pinMode(DAT, INPUT);
  pinMode(RST, OUTPUT);
  digitalWrite(CLK, LOW);
  digitalWrite(RST, LOW);
  initialized = 1;

  return 0;
}

uint8_t N51PGM_is_init(void){
  // check what the pins are set to
  return initialized;
}

void N51PGM_set_dat(uint8_t val)
{
  digitalWrite(DAT, val);
}

uint8_t N51PGM_get_dat(void)
{
  return digitalRead(DAT);
}

void N51PGM_set_rst(uint8_t val)
{
  digitalWrite(RST, val);
}

void N51PGM_set_clk(uint8_t val)
{
  digitalWrite(CLK, val);
}

void N51PGM_set_target_power(uint8_t on)
{
  N51PGM_init_power_pin();
  if (!on) {
    N51PGM_release_pins();
    initialized = 0;
  }
#if TARGET_POWER_ACTIVE_HIGH
  digitalWrite(PWR, on ? HIGH : LOW);
#else
  digitalWrite(PWR, on ? LOW : HIGH);
#endif
}

void N51PGM_dat_dir(uint8_t state)
{
  pinMode(DAT, state ? OUTPUT : INPUT);
}

void N51PGM_release_pins(void)
{
  pinMode(CLK, INPUT);
  pinMode(DAT, INPUT);
  pinMode(RST, INPUT);
}

void N51PGM_set_trigger(uint8_t val)
{
  /* not implemented */
}

void N51PGM_release_rst(void)
{
  pinMode(RST, INPUT);
}

void N51PGM_deinit(uint8_t leave_reset_high)
{
  pinMode(CLK, INPUT);
  pinMode(DAT, INPUT);
  if (leave_reset_high){
    N51PGM_set_rst(1);
  } else {
    pinMode(RST, INPUT);
  }
  initialized = 0;
}


#ifdef TEST_USLEEP
#include <stdarg.h>
void N51PGM_debug_outputf(const char *s, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, s);
  vsnprintf(buf, 160, s, ap);
  N51PGM_print(buf);

  va_end(ap);
}

#define DEBUG_OUTPUTF(s, ...) N51PGM_debug_outputf(s, ##__VA_ARGS__)
#else
#define DEBUG_OUTPUTF(s, ...)
#endif
uint32_t N51PGM_usleep(uint32_t usec)
{
  if (usec < 1000) {
    delayMicroseconds(usec);
    // not printing for short delays to avoid potentially destructive overhead
  } else {
    uint32_t msec = usec / 1000;
    uint32_t lusec = usec % 1000;
    DEBUG_OUTPUTF("usleep(%u): ", usec);
    DEBUG_OUTPUTF("delaying %u ms, ", msec); 
    DEBUG_OUTPUTF("%u us\n", lusec);
    delay(msec);
    delayMicroseconds(lusec);
  }
  return usec;
}

uint64_t N51PGM_get_time(){
    return micros();
}

void N51PGM_print(const char * msg){
#ifdef _DEBUG
    DebugSerial.print(msg);
#endif
}

} // extern "C"
