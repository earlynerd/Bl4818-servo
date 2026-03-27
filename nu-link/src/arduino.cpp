#include <Arduino.h>
#define _DEBUG
#include "n51_pgm.h"

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

  pinMode(N51PGM_PWR_PIN, OUTPUT);
#if TARGET_POWER_ACTIVE_HIGH
  digitalWrite(N51PGM_PWR_PIN, TARGET_POWER_DEFAULT_ON ? HIGH : LOW);
#else
  digitalWrite(N51PGM_PWR_PIN, TARGET_POWER_DEFAULT_ON ? LOW : HIGH);
#endif
  power_pin_initialized = 1;
}

int N51PGM_init(void)
{
  N51PGM_init_power_pin();
  pinMode(N51PGM_CLK_PIN, OUTPUT);
  pinMode(N51PGM_DAT_PIN, INPUT);
  pinMode(N51PGM_RST_PIN, OUTPUT);
  digitalWrite(N51PGM_CLK_PIN, LOW);
  digitalWrite(N51PGM_RST_PIN, LOW);
  initialized = 1;

  return 0;
}

uint8_t N51PGM_is_init(void){
  // check what the pins are set to
  return initialized;
}

void N51PGM_set_dat(uint8_t val)
{
  digitalWrite(N51PGM_DAT_PIN, val);
}

uint8_t N51PGM_get_dat(void)
{
  return digitalRead(N51PGM_DAT_PIN);
}

void N51PGM_set_rst(uint8_t val)
{
  digitalWrite(N51PGM_RST_PIN, val);
}

void N51PGM_set_clk(uint8_t val)
{
  digitalWrite(N51PGM_CLK_PIN, val);
}

void N51PGM_set_target_power(uint8_t on)
{
  N51PGM_init_power_pin();
  if (!on) {
    N51PGM_release_pins();
    initialized = 0;
  }
#if TARGET_POWER_ACTIVE_HIGH
  digitalWrite(N51PGM_PWR_PIN, on ? HIGH : LOW);
#else
  digitalWrite(N51PGM_PWR_PIN, on ? LOW : HIGH);
#endif
}

void N51PGM_dat_dir(uint8_t state)
{
  pinMode(N51PGM_DAT_PIN, state ? OUTPUT : INPUT);
}

void N51PGM_release_pins(void)
{
  pinMode(N51PGM_CLK_PIN, INPUT);
  pinMode(N51PGM_DAT_PIN, INPUT);
  pinMode(N51PGM_RST_PIN, INPUT);
}

void N51PGM_set_trigger(uint8_t val)
{
  /* not implemented */
}

void N51PGM_release_rst(void)
{
  pinMode(N51PGM_RST_PIN, INPUT);
}

void N51PGM_deinit(uint8_t leave_reset_high)
{
  pinMode(N51PGM_CLK_PIN, INPUT);
  pinMode(N51PGM_DAT_PIN, INPUT);
  if (leave_reset_high){
    N51PGM_set_rst(1);
  } else {
    pinMode(N51PGM_RST_PIN, INPUT);
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
