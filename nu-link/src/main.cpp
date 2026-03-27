#define _DEBUG

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <stdint.h>
#include "n51_icp.h"
#include "n51_pgm.h"
#include "config.h"


#include <assert.h>
#include "isp_common.h"
#include "device_common.h"

Adafruit_USBD_CDC DebugSerial;

#define FW_VERSION          0xE1  // Bridge FW with target-power recovery commands

// We refuse to set configs that have the reset pin disabled and the watchdog timer disabled
// It becomes very, very difficult to re-flash the device if the reset pin is disabled AND it doesn't reset on a periodic basis
#define NO_DANGEROUS_CONFIGS 1

// Change this setting if your target board doesn't have reset set to high by default
// 0: sets Reset pin to high-impedence (i.e. neutral) after programming
// 1: leaves Reset pin high after programming
#define LEAVE_RESET_HIGH 0

#ifdef ARDUINO_AVR_MEGA2560
#define CACHED_ROM_READ 0
#endif
// NOTE: If your sketch ends up being too big to fit on the device, you can try setting CACHED_ROM_READ to 0
#ifndef CACHED_ROM_READ
#define CACHED_ROM_READ 1
#endif
// connection timeout in milliseconds; 0 to disable
#define CONNECTION_TIMEOUT 0

#define DEBUG_VERBOSE 1

#ifndef USING_32BIT_PACKNO
#define USING_32BIT_PACKNO 1
#endif

#ifndef BUILTIN_LED
#define BUILTIN_LED LED_BUILTIN
#endif

#define TARGET_UART_DEFAULT_BAUD 115200
#define TARGET_UART_FIFO_SIZE    256


#define PAGE_SIZE            128 // flash page size
#define PAGE_MASK            0xFF80
#define MAX_FLASH_SIZE  (64 * 1024) // 64k
// N76E003 device constants
#define N76E003_DEVID	         0x3650
#define MS16K_512B_DEVID         0x4B20
#define MS16K_1K_OLD_DEVID       0x4B21
#define MS16K_1K_DEVID           0x5322
#define CFG_FLASH_ADDR		       0x30000
#define CFG_FLASH_LEN		         5
#define LDROM_MAX_SIZE      (4 * 1024)



#define DISCONNECTED_STATE      0
#define CONNECTING_STATE        1
#define WAITING_FOR_CONNECT_CMD 2
#define WAITING_FOR_SYNCNO      3
#define COMMAND_STATE           4
#define UPDATING_STATE          5
#define DUMPING_STATE           6

uint8_t state;
unsigned char rx_buf[PACKSIZE];
unsigned char tx_buf[PACKSIZE];
int rx_bufhead = 0;
uint32_t g_packno = 0;
int update_addr = 0x0000;
uint32_t update_size = 0;
uint16_t g_update_checksum = 0;
int dump_addr = 0x0000;
uint32_t dump_size = 0;
uint8_t cid;
uint32_t saved_device_id;
uint8_t connected = 0;
uint8_t just_connected = 0;
unsigned long last_read_time = 0;
unsigned long curr_time = 0;
static SerialPIO target_uart(N51PGM_CLK_PIN, N51PGM_DAT_PIN, TARGET_UART_FIFO_SIZE);
static bool target_uart_running = false;
static uint32_t target_uart_baud = 0;
static uint16_t target_uart_config = SERIAL_8N1;

#if CACHED_ROM_READ
byte read_buff[MAX_FLASH_SIZE];
bool read_buff_valid = false;
#define INVALIDATE_CACHE read_buff_valid = false
#else
#define INVALIDATE_CACHE
#endif




#define XSTR(x) STR(x)
#define STR(x) #x

void read_config(config_flags *flags);
void fail_pkt();
int get_ldrom_size(config_flags *flags);
static bool is_unsynced_cmd(int cmd);
static void write_u32_to_tx(uint32_t value);
static bool enter_icp_after_power_cycle(uint16_t off_ms, uint16_t delay_us);
static uint16_t get_debug_uart_config(void);
static bool debug_passthrough_allowed(void);
static void stop_debug_passthrough(void);
static void service_debug_passthrough(void);


#ifdef _DEBUG
#pragma message "DEBUG MODE!!!"
#define DEBUG_PRINT(...) N51ICP_outputf(__VA_ARGS__)
#define DEBUG_PRINT_BYTEARR(arr, len) \
  for (int i = 0; i < len; i++) \
    N51ICP_outputf(" %02x", (arr)[i]); \
  N51ICP_outputf("\n");


static unsigned long usstart_time = 0;
static unsigned long usend_time = 0;
#define TIMER_START usstart_time = N51PGM_get_time();
#define TIMER_END usend_time = N51PGM_get_time();
#define PRINT_TIME(funcname) N51ICP_outputf(#funcname " took %lu us\n", usend_time - usstart_time)
#define TIME_FUNCTION(funcname, ...) \
		TIMER_START; \
		funcname(__VA_ARGS__); \
		TIMER_END; \
		DEBUG_PRINT(#funcname " took %lu us\n", usend_time - usstart_time);

// #define TEST_USLEEP 1
#ifdef TEST_USLEEP

void _debug_outputf(const char *s, ...)
{
  char buf[160];
  va_list ap;
  va_start(ap, s);
  vsnprintf(buf, 160, s, ap);
  va_end(ap);
  DebugSerial.print(buf);
}

// #define USLEEP(x) \
// 	TIMER_START; \
// 	if (x > 0) N51PGM_usleep(x);\
// 	TIMER_END; \
// 	DEBUG_PRINT("USLEEP(%lu) took %lu us\n", x, usend_time - usstart_time);
#define USLEEP(x) \
	if (x > 0) {\
		if (x < 500){\
			N51PGM_usleep(x);\
		}\
		else { \
			TIMER_START; \
			N51PGM_usleep(x);\
			TIMER_END; \
			N51ICP_outputf("USLEEP(%lu) took %lu us\n", x, usend_time - usstart_time);\
		}\
	};

void test_usleep() {
#pragma message "Testing usleep"
  for (uint32_t i = 0; i < 500; i+=10){

    USLEEP(i);
  }
  for (uint32_t usec = 1000; usec < 60000; usec+=1000){
    
    uint32_t lusec = usec % 1000;
    uint32_t msec = usec / 1000;
    uint32_t blurgh = 0;
    DEBUG_PRINT("!!!usleep(%u): ", usec);
    DEBUG_PRINT("delaying %u ms, ", msec); 
    DEBUG_PRINT("%u us\n", lusec);
    USLEEP(usec);
  }
}
#endif // TEST_USLEEP

#else // _DEBUG
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_BYTEARR(arr, len)
#endif // _DEBUG

// implementation specific
void enable_connect_led(){
  digitalWrite(BUILTIN_LED, HIGH);
}

// implementation specific
void disable_connect_led(){
  digitalWrite(BUILTIN_LED, LOW);
}

// implementation specific
void setup()
{
  Serial.begin(115200);
  DebugSerial.begin(115200);
  while(!DebugSerial);
  delay(100);
  DebugSerial.println("Debug port output prints below...");
  pinMode(BUILTIN_LED, OUTPUT);
  N51PGM_set_target_power(1);
  disable_connect_led();
  state = DISCONNECTED_STATE;
  memset(rx_buf, (uint8_t)0xFF, PACKSIZE);
  memset(tx_buf, (uint8_t)0xFF, PACKSIZE);

#ifdef _DEBUG
  delay(100);
  DebugSerial.begin(115200);
  while (!DebugSerial); // wait for serial port to connect. Needed for native USB port only
  DebugSerial.println("online");
#ifdef TEST_USLEEP
  test_usleep();
#endif
#ifdef DEBUG_START_PRINT
  N51ICP_init();
  N51ICP_enter_icp_mode(true);
  DEBUG_PRINT("DEVICEID\t\t\t0x%02x\n", N51ICP_read_device_id());
  DEBUG_PRINT("CID\t\t\t0x%02x\n", N51ICP_read_cid());
  DEBUG_PRINT("UID\t\t\t0x%024x\n", N51ICP_read_uid());
  DEBUG_PRINT("UCID\t\t\t0x%032x\n", N51ICP_read_ucid());
  uint8_t buf[16];
  uint16_t addr = 0;
  while (addr < 256) {
    N51ICP_read_flash(addr, sizeof(buf), buf);
    DEBUG_PRINT("%04x: ", addr);
    for (int i = 0; i < sizeof(buf); i++)
      DEBUG_PRINT("%02x ", buf[i]);
    DEBUG_PRINT("\n");
    addr += sizeof(buf);
  }
  N51ICP_exit_icp_mode();
  N51ICP_deinit(LEAVE_RESET_HIGH);
#endif // DEBUG_START_PRINT
#endif // _DEBUG
}



void inc_g_packno(){
  g_packno++;
}

// implementation specific
void tx_pkt()
{
  DEBUG_PRINT("Sending packet...\n");
#if DEBUG_VERBOSE
  for (int i = 0; i < PACKSIZE; i++){
    DEBUG_PRINT(" %02x", tx_buf[i]);
  }
  DEBUG_PRINT("\n");
#endif
  uint8_t pktsize = 0;
  while (pktsize < PACKSIZE)
    Serial.write(tx_buf[pktsize++]);
  DEBUG_PRINT("done sending packet\n");
}

uint16_t get_checksum() {
  uint16_t checksum = 0;
  for (int i = 0; i < PACKSIZE; i++)
    checksum += rx_buf[i];
  return checksum;
}

uint16_t package_checksum() {
  uint16_t checksum = get_checksum();
  tx_buf[0] = checksum & 0xff;
  tx_buf[1] = (checksum >> 8) & 0xff;
  tx_buf[2] = 0;
  tx_buf[3] = 0;
  return checksum;
}

void prep_pkt() {
  // populate header
  package_checksum();
  inc_g_packno();
  tx_buf[4] = g_packno & 0xff;
  tx_buf[5] = (g_packno >> 8) & 0xff;
#ifdef USING_32BIT_PACKNO
  tx_buf[6] = (g_packno >> 16) & 0xff;
  tx_buf[7] = (g_packno >> 24) & 0xff;
#else
  tx_buf[6] = 0;
  tx_buf[7] = 0;
#endif
}

void send_pkt() {
  prep_pkt();
  tx_pkt();
}


void update(unsigned char* data, int len)
{
  int n = len > update_size ? update_size : len;
  DEBUG_PRINT("writing %d bytes to flash at addr 0x%04x\n", n, update_addr);
  update_addr = N51ICP_write_flash(update_addr, n, data);
  // update the checksum
  for (int i = 0; i < n; i++)
    g_update_checksum += data[i];
  update_size -= n;
}



uint32_t get_aprom_size(){
  config_flags flags;
  read_config(&flags);
  const flash_info_t * flash_info = get_flash_info(N51ICP_read_device_id());
  if (flash_info == NULL) {
    return 0;
  }
  return flash_info_get_aprom_size(flash_info, get_ldrom_size(&flags));
}

uint32_t get_flash_size() {
  const flash_info_t * flash_info = get_flash_info(N51ICP_read_device_id());
  if (flash_info == NULL) {
    return 0;
  }
  return flash_info->memory_size;
}

static bool is_range_valid(uint32_t addr, uint32_t size, uint32_t limit, const char *label) {
  if (size == 0) {
    DEBUG_PRINT("%s write rejected: zero-length request\n", label);
    return false;
  }
  if (limit == 0) {
    DEBUG_PRINT("%s write rejected: unknown flash size\n", label);
    return false;
  }
  if (addr >= limit) {
    DEBUG_PRINT("%s write rejected: start 0x%04lx is outside limit 0x%04lx\n", label, (unsigned long)addr, (unsigned long)limit);
    return false;
  }
  if (size > (limit - addr)) {
    DEBUG_PRINT("%s write rejected: range 0x%04lx..0x%04lx exceeds limit 0x%04lx\n",
      label,
      (unsigned long)addr,
      (unsigned long)(addr + size - 1),
      (unsigned long)(limit - 1));
    return false;
  }
  return true;
}

static bool is_aprom_range_valid(uint32_t addr, uint32_t size) {
  return is_range_valid(addr, size, get_aprom_size(), "APROM");
}

static bool is_nvm_range_valid(uint32_t addr, uint32_t size) {
  return is_range_valid(addr, size, get_flash_size(), "whole-ROM");
}

static bool is_whole_rom_update_valid(uint32_t addr, uint32_t size) {
  config_flags flags;
  read_config(&flags);
  if (get_ldrom_size(&flags) != 0) {
    DEBUG_PRINT("whole-ROM write rejected: LDROM is reserved; use APROM-only update commands\n");
    return false;
  }
  return is_nvm_range_valid(addr, size);
}

static bool is_config_safe(const config_flags *flags) {
#if NO_DANGEROUS_CONFIGS
  config_flags decoded = *flags;
  if (flags->RPD == 0) {
    DEBUG_PRINT("Refusing config write with reset pin disabled\n");
    return false;
  }
  if (flags->CBS == 0 && get_ldrom_size(&decoded) == 0) {
    DEBUG_PRINT("Refusing config write that boots LDROM when no LDROM is reserved\n");
    return false;
  }
#endif
  return true;
}

static bool write_config_checked(const uint8_t *config_bytes) {
  uint8_t verify_buf[CFG_FLASH_LEN];
  N51ICP_page_erase(CFG_FLASH_ADDR);
  N51ICP_write_flash(CFG_FLASH_ADDR, CFG_FLASH_LEN, (uint8_t *)config_bytes);
  N51ICP_read_flash(CFG_FLASH_ADDR, CFG_FLASH_LEN, verify_buf);
  if (memcmp(verify_buf, config_bytes, CFG_FLASH_LEN) != 0) {
    DEBUG_PRINT("Config write verification failed\n");
    return false;
  }
  return true;
}

void dump()
{
  unsigned char * data_buf = tx_buf + DUMP_DATA_START;
  int n = DUMP_DATA_SIZE > dump_size ? dump_size : DUMP_DATA_SIZE;

  // uint16_t checksum = 0;

#if CACHED_ROM_READ
  // hack to make reads faster
  if (!read_buff_valid) {
    uint32_t device_id = N51ICP_read_device_id();
    const flash_info_t * flash_info = get_flash_info(device_id);
    if (flash_info == NULL) {
      DEBUG_PRINT("Failed to get flash info for device 0x%04x\n", N51ICP_read_device_id());
      fail_pkt();
      return;
    }
    DEBUG_PRINT("Caching rom...\n");
    // we're going to read the entire thing into memory
    uint32_t read_chunk = flash_info->memory_size;
    N51ICP_read_flash(0, flash_info->memory_size, read_buff);
    read_buff_valid = true;
  }
  memcpy(data_buf, &read_buff[dump_addr], n);
  dump_addr += n;
#else
  dump_addr = N51ICP_read_flash(dump_addr, n, data_buf);
#endif
  dump_size -= n;
}


#ifdef _DEBUG
const char * cmd_enum_to_string(int cmd)
{
  switch (cmd) {
    case CMD_UPDATE_APROM: return "CMD_UPDATE_APROM";
    case CMD_UPDATE_CONFIG: return "CMD_UPDATE_CONFIG";
    case CMD_READ_CONFIG: return "CMD_READ_CONFIG";
    case CMD_ERASE_ALL: return "CMD_ERASE_ALL";
    case CMD_SYNC_PACKNO: return "CMD_SYNC_PACKNO";
    case CMD_GET_FWVER: return "CMD_GET_FWVER";
    case CMD_RUN_APROM: return "CMD_RUN_APROM";
    case CMD_RUN_LDROM: return "CMD_RUN_LDROM";
    case CMD_CONNECT: return "CMD_CONNECT";
    case CMD_GET_DEVICEID: return "CMD_GET_DEVICEID";
    case CMD_RESET: return "CMD_RESET";
    case CMD_GET_FLASHMODE: return "CMD_GET_FLASHMODE";
    case CMD_UPDATE_WHOLE_ROM: return "CMD_UPDATE_WHOLE_ROM";
    case CMD_WRITE_CHECKSUM: return "CMD_WRITE_CHECKSUM";
    case CMD_RESEND_PACKET: return "CMD_RESEND_PACKET";
    case CMD_READ_ROM: return "CMD_READ_ROM";
    case CMD_GET_UID: return "CMD_GET_UID";
    case CMD_GET_CID: return "CMD_GET_CID";
    case CMD_GET_UCID: return "CMD_GET_UCID";
    case CMD_ISP_PAGE_ERASE: return "CMD_ISP_PAGE_ERASE";
    case CMD_ISP_MASS_ERASE: return "CMD_ISP_MASS_ERASE";
    case CMD_TARGET_POWER: return "CMD_TARGET_POWER";
    case CMD_POWER_CYCLE_CONNECT: return "CMD_POWER_CYCLE_CONNECT";
    default: return "UNKNOWN";
  }
}

#endif

void fail_pkt(){
  DEBUG_PRINT("Sending fail packet\n");
  prep_pkt();
  tx_buf[0] = ~tx_buf[0];
  tx_buf[1] = ~tx_buf[1];
  tx_pkt();
}

bool mass_erase_checked(bool check_device_id = false){
  INVALIDATE_CACHE;
  uint8_t cid = N51ICP_read_cid();
  N51ICP_mass_erase();
  N51PGM_usleep(500000); // half a second
  if (cid == 0xFF || cid == 0x00){
    N51ICP_reentry(5000, 1000, 10);
  }
  if (check_device_id){
    if (get_flash_info(N51ICP_read_device_id()) == NULL){
      N51ICP_reentry(5000, 1000, 10);
      if (get_flash_info(N51ICP_read_device_id()) == NULL) {
        DEBUG_PRINT("Failed to find device after mass erase! failing...\n");
        fail_pkt();
        return false;
      }
    }
  }
  return true;
}

static bool is_unsynced_cmd(int cmd) {
  return cmd == CMD_CONNECT || cmd == CMD_POWER_CYCLE_CONNECT || cmd == CMD_TARGET_POWER;
}

static void write_u32_to_tx(uint32_t value) {
  tx_buf[8] = value & 0xff;
  tx_buf[9] = (value >> 8) & 0xff;
  tx_buf[10] = (value >> 16) & 0xff;
  tx_buf[11] = (value >> 24) & 0xff;
}

static bool enter_icp_after_power_cycle(uint16_t off_ms, uint16_t delay_us) {
  N51ICP_deinit(0);
  N51PGM_set_target_power(0);
  delay(off_ms);
  N51PGM_set_target_power(1);

  if (N51ICP_init() != 0) {
    DEBUG_PRINT("Failed to initialize the PGM after power cycle\n");
    return false;
  }

  saved_device_id = N51ICP_enter_icp_after_powerup(delay_us);
  DEBUG_PRINT("Power-cycle connect device ID: 0x%04lx\n", saved_device_id);
  return get_flash_info(saved_device_id) != NULL;
}

static uint16_t get_debug_uart_config(void) {
  uint16_t config = 0;

  switch (DebugSerial.numbits()) {
    case 5:
      config |= SERIAL_DATA_5;
      break;
    case 6:
      config |= SERIAL_DATA_6;
      break;
    case 7:
      config |= SERIAL_DATA_7;
      break;
    default:
      config |= SERIAL_DATA_8;
      break;
  }

  switch (DebugSerial.paritytype()) {
    case 1: /* CDC odd parity */
      config |= SERIAL_PARITY_ODD;
      break;
    case 2: /* CDC even parity */
      config |= SERIAL_PARITY_EVEN;
      break;
    default:
      config |= SERIAL_PARITY_NONE;
      break;
  }

  if (DebugSerial.stopbits() == 2) {
    config |= SERIAL_STOP_BIT_2;
  } else {
    config |= SERIAL_STOP_BIT_1;
  }

  return config;
}

static bool debug_passthrough_allowed(void) {
  return state == DISCONNECTED_STATE && rx_bufhead == 0 && !N51PGM_is_init();
}

static void stop_debug_passthrough(void) {
  if (!target_uart_running) {
    return;
  }

  target_uart.end();
  target_uart_running = false;
}

static void service_debug_passthrough(void) {
  uint32_t baud;
  uint16_t config;

  if (!debug_passthrough_allowed() || !DebugSerial.dtr()) {
    stop_debug_passthrough();
    return;
  }

  baud = DebugSerial.baud();
  if (baud == 0) {
    baud = TARGET_UART_DEFAULT_BAUD;
  }
  config = get_debug_uart_config();

  if (!target_uart_running || baud != target_uart_baud || config != target_uart_config) {
    stop_debug_passthrough();
    target_uart_baud = baud;
    target_uart_config = config;
    target_uart.begin(target_uart_baud, target_uart_config);
    target_uart_running = (bool)target_uart;
    if (!target_uart_running) {
      return;
    }
  }

  while (DebugSerial.available() && target_uart.availableForWrite() > 0) {
    int ch = DebugSerial.read();
    if (ch < 0) {
      break;
    }
    target_uart.write((uint8_t)ch);
  }

  while (target_uart.available() && DebugSerial.availableForWrite() > 0) {
    int ch = target_uart.read();
    if (ch < 0) {
      break;
    }
    DebugSerial.write((uint8_t)ch);
  }
}


void read_config(config_flags *flags) {
  N51ICP_read_flash(CFG_FLASH_ADDR, CFG_FLASH_LEN, (uint8_t *)flags);
}
int get_ldrom_size(config_flags *flags){
  return (flags->LDS < 3 ? 4 : (7 - flags->LDS)) * 1024;
}

int read_ldrom_size() {
  config_flags flags;
  read_config(&flags);
  return get_ldrom_size(&flags);
}

void start_dump(int addr, int size){
  config_flags flags;
  read_config(&flags);
  uint8_t cid = N51ICP_read_cid();
  if (cid == 0xFF || cid == 0x00){
    // attempt reentry if lock bit is unlocked
    if (flags.LOCK == 1) {
      N51ICP_reentry(5000, 1000, 10);
      cid = N51ICP_read_cid(); 
    }
    if (cid == 0xFF || cid == 0x00) {
      DEBUG_PRINT("Device is locked, cannot dump\n");
      DEBUG_PRINT("CID is 0x%02x\n", cid);
      DEBUG_PRINT("LOCK bit = %d (%s)\n", flags.LOCK, flags.LOCK ? "unlocked" : "locked");
      fail_pkt();
      return;
    }
  }
  if (flags.LOCK == 0) {
    DEBUG_PRINT("WARNING: lock bit is locked, but cid indicates still in an unlocked state, attempting dump anyway...\n");
  }

  dump_addr = addr;
  dump_size = size;
  
  dump();
  if (dump_size > 0)
    state = DUMPING_STATE;
  send_pkt();
}

void reset_buf() {
  rx_bufhead = 0;
}
void reset_conn() {
  DEBUG_PRINT("Disconnecting...\n");
  if (state > WAITING_FOR_CONNECT_CMD) {
    disable_connect_led();
    N51ICP_exit_icp_mode();
    N51ICP_deinit(LEAVE_RESET_HIGH);
  }
  state = DISCONNECTED_STATE;
}


void add_g_total_checksum(){
  // Specification is unclear about how long the checksum is supposed to be; we assume 16-bit
  tx_buf[8] = g_update_checksum & 0xff;
  tx_buf[9] = (g_update_checksum >> 8) & 0xff;
  tx_buf[10] = 0;
  tx_buf[11] = 0;
}

bool check_packet_timeout(){
  return curr_time - last_read_time > 500;
}

void loop()
{
  curr_time = millis();
  if (Serial.available()) {
    stop_debug_passthrough();
    int tmp = Serial.read();
    rx_buf[rx_bufhead++] = tmp;
    if (state == DISCONNECTED_STATE) {
      if (!is_unsynced_cmd(tmp)){
        DEBUG_PRINT("NOCONN: %d\n", tmp);
        reset_buf();
        return;
      }
      state = CONNECTING_STATE;
    } else if (state == CONNECTING_STATE) {
      if (rx_bufhead < 5) {
        if (tmp != 0){
          DEBUG_PRINT("0NOT\n");
          state = DISCONNECTED_STATE;
          reset_buf();
          return;
        }
      } else {
        state = WAITING_FOR_CONNECT_CMD;
      }
    }

    if (rx_bufhead < PACKSIZE) {
      return;
    }
    DEBUG_PRINT("received packet\n");
    // full packet received
    last_read_time = millis();
    rx_bufhead = 0;
    inc_g_packno();
#if DEBUG_VERBOSE
    DEBUG_PRINT("received packet: ");
    for (int i = 0; i < PACKSIZE; i++)
      DEBUG_PRINT(" %02x", rx_buf[i]);
    DEBUG_PRINT("\n");
#endif
    
    uint8_t cid;
    uint32_t devid;
    uint8_t cmd = rx_buf[0];
    uint32_t seqno = (rx_buf[5] << 8) | rx_buf[4];
    int num_read = 0;
    int ldrom_size = 0;
    config_flags flags;

    DEBUG_PRINT("received %d-byte packet, %s (0x%02x), seqno 0x%04x, checksum 0x%04x\n", PACKSIZE, cmd_enum_to_string(cmd), cmd, seqno, get_checksum());

#if CHECK_SEQUENCE_NO
    if (g_packno != seqno && cmd != CMD_SYNC_PACKNO && !is_unsynced_cmd(cmd))
    {
      DEBUG_PRINT("seqno mismatch, expected 0x%04x, got 0x%04x, ignoring packet...\n", g_packno, seqno);
      state = COMMAND_STATE;
      send_pkt();
      return;
    }
#endif
    if (state == WAITING_FOR_SYNCNO && cmd != CMD_SYNC_PACKNO && !is_unsynced_cmd(cmd)) {
      // No syncno command, just skip to command state
      state = COMMAND_STATE;
    } else if ((state == DUMPING_STATE || state == UPDATING_STATE) && cmd != CMD_FORMAT2_CONTINUATION) {
      state = COMMAND_STATE;
    } else if (state == DUMPING_STATE) {
      dump();
      if (dump_size == 0)
        state = COMMAND_STATE;
      send_pkt();
      return;
    } else if (state == UPDATING_STATE) {
      update(&rx_buf[8], SEQ_UPDATE_PKT_SIZE);
      if (update_size == 0) {
        state = COMMAND_STATE;
      }
      add_g_total_checksum();
      send_pkt();
      return;
    }
    switch (cmd) {
      case CMD_CONNECT:
        {
          g_packno = 0;
          DEBUG_PRINT("CMD_CONNECT\n");
          INVALIDATE_CACHE;
          if (state == WAITING_FOR_CONNECT_CMD) {
            state = WAITING_FOR_SYNCNO;
            if (N51ICP_init() != 0) {
              DEBUG_PRINT("Failed to initialize the PGM\n");
              state = DISCONNECTED_STATE;
              fail_pkt();
              return;
            }
            saved_device_id = N51ICP_enter_icp_mode(true);
            enable_connect_led();
          } else if (state == WAITING_FOR_SYNCNO) {
            // Don't send back a packet if we just connected and are waiting for syncno
            // It means that we got multiple connect commands, we only need to respond to one of them
            break;
          }
          write_u32_to_tx(saved_device_id);
          memset(&tx_buf[12], 0, PACKSIZE - 12);
          send_pkt();
          DEBUG_PRINT("Connected!\n");
        } break;
      case CMD_POWER_CYCLE_CONNECT:
        {
          g_packno = 0;
          uint16_t off_ms = rx_buf[8] | (rx_buf[9] << 8);
          uint16_t delay_us = rx_buf[10] | (rx_buf[11] << 8);
          DEBUG_PRINT("CMD_POWER_CYCLE_CONNECT (off_ms=%u, delay_us=%u)\n", off_ms, delay_us);
          INVALIDATE_CACHE;
          if (state == WAITING_FOR_CONNECT_CMD) {
            state = WAITING_FOR_SYNCNO;
            if (!enter_icp_after_power_cycle(off_ms, delay_us)) {
              DEBUG_PRINT("Power-cycle connect failed\n");
              state = DISCONNECTED_STATE;
              fail_pkt();
              return;
            }
            enable_connect_led();
          } else if (state == WAITING_FOR_SYNCNO) {
            break;
          }
          write_u32_to_tx(saved_device_id);
          memset(&tx_buf[12], 0, PACKSIZE - 12);
          send_pkt();
          DEBUG_PRINT("Connected!\n");
        } break;
      case CMD_TARGET_POWER:
        {
          uint8_t power_on = rx_buf[8] ? 1 : 0;
          DEBUG_PRINT("CMD_TARGET_POWER (%s)\n", power_on ? "on" : "off");
          if (!power_on && state > WAITING_FOR_CONNECT_CMD) {
            disable_connect_led();
            N51ICP_deinit(0);
          }
          N51PGM_set_target_power(power_on);
          if (!power_on || state < WAITING_FOR_SYNCNO) {
            state = DISCONNECTED_STATE;
          }
          tx_buf[8] = power_on;
          tx_buf[9] = 0;
          tx_buf[10] = 0;
          tx_buf[11] = 0;
          send_pkt();
        } break;
      case CMD_GET_FWVER:
        tx_buf[8] = FW_VERSION;
        tx_buf[9] = 0;
        tx_buf[10] = 0;
        tx_buf[11] = 0;
        send_pkt();
        break;
      case CMD_GET_FLASHMODE:
        DEBUG_PRINT("CMD_GET_FLASHMODE\n");
        read_config(&flags);
        if (flags.CBS == 1){
          tx_buf[8] = APMODE;
        } else {
          tx_buf[8] = LDMODE;
        }
        tx_buf[9] = 0;
        tx_buf[10] = 0;
        tx_buf[11] = 0;
        send_pkt();
        break;
      case CMD_SYNC_PACKNO:
      {
        DEBUG_PRINT("CMD_SYNC_PACKNO\n");
#if CHECK_SEQUENCE_NO
        int seqnoCopy = (rx_buf[9] << 8) | rx_buf[8];
        if (seqnoCopy != seqno)
        {
          DEBUG_PRINT("seqno mismatch, expected 0x%04x, got 0x%04x, ignoring packet...\n", seqno, seqnoCopy);
          g_packno = -1; // incremented by send_pkt
        }
        else
#endif
        {
          g_packno = seqno;
        }
        state = COMMAND_STATE;
        send_pkt();
      }
      break;
      case CMD_GET_CID:
        {
        DEBUG_PRINT("CMD_GET_CID\n");
        uint8_t id = N51ICP_read_cid();
        DEBUG_PRINT("received cid of 0x%02x\n", id);
        tx_buf[8] = id;
        tx_buf[9] = 0;
        tx_buf[10] = 0;
        tx_buf[11] = 0;
        send_pkt();
        } break;
      case CMD_GET_UID:
        {
        N51ICP_read_uid(&tx_buf[8]);
        DEBUG_PRINT("received uid of ");
        DEBUG_PRINT_BYTEARR(&tx_buf[8], 12);
        send_pkt();
        } break;
      case CMD_GET_UCID:
        {
        // __uint128_t id = N51ICP_read_ucid();
        // DEBUG_PRINT("received ucid of 0x%08x\n", id);
        // for (int i = 0; i < 16; i++)
        //   rx_buf[8 + i] = (id >> (i * 8)) & 0xff;
        N51ICP_read_ucid(&tx_buf[8]);
        DEBUG_PRINT("received ucid of ");
        DEBUG_PRINT_BYTEARR(&tx_buf[8], 16);
        send_pkt();
        } break;
      case CMD_GET_DEVICEID:
        {
        uint32_t id = N51ICP_read_device_id();
        DEBUG_PRINT("received device id of 0x%04x\n", id);
        tx_buf[8] = id & 0xff;
        tx_buf[9] = (id >> 8) & 0xff;
        tx_buf[10] = 0;
        tx_buf[11] = 0;
        send_pkt();
        } break;
      case CMD_GET_PID:
        {
        uint32_t id = N51ICP_read_pid();
        DEBUG_PRINT("received part id of 0x%04x\n", id);
        tx_buf[8] = id & 0xff;
        tx_buf[9] = (id >> 8) & 0xff;
        tx_buf[10] = 0;
        tx_buf[11] = 0;
        send_pkt();
        } break;
      case CMD_READ_CONFIG:
        DEBUG_PRINT("CMD_READ_CONFIG\n");
        N51ICP_read_flash(CFG_FLASH_ADDR, CFG_FLASH_LEN, &tx_buf[8]);
        // set the rest of the packet to FF
        memset(&tx_buf[8 + CFG_FLASH_LEN], 0xFF, PACKSIZE - 8 - CFG_FLASH_LEN);
        send_pkt();
        break;
      case CMD_UPDATE_CONFIG: {
        DEBUG_PRINT("CMD_UPDATE_CONFIG\n");
        INVALIDATE_CACHE;
        const config_flags * update_flags = (const config_flags *)&rx_buf[8];
        if (!is_config_safe(update_flags)) {
          fail_pkt();
          break;
        }
        if (!write_config_checked(&rx_buf[8])) {
          fail_pkt();
          break;
        }
        send_pkt();
      } break;
      case CMD_ERASE_ALL: // Erase all only erases the AP ROM, so we have to page erase the APROM area
      {
        DEBUG_PRINT("CMD_ERASE_ALL\n");
        INVALIDATE_CACHE;
        // read_config(&flags);
        // int ldrom_size = get_ldrom_size(&flags);
        // DEBUG_PRINT("ldrom_size: %d\n", ldrom_size);
        uint32_t aprom_size = get_aprom_size();
        if (aprom_size == 0) {
          DEBUG_PRINT("Erase rejected: unknown APROM size\n");
          fail_pkt();
          break;
        }
        DEBUG_PRINT("Erasing %d bytes of APROM\n", aprom_size);
        for (uint32_t i = 0; i < aprom_size; i += PAGE_SIZE) {
          N51ICP_page_erase(i);
        }
        send_pkt();
      } break;
      case CMD_ISP_MASS_ERASE:
        {
        DEBUG_PRINT("CMD_ISP_MASS_ERASE\n");

          INVALIDATE_CACHE;
          if (!mass_erase_checked(false)) break;
          send_pkt();
        }
        break;
      case CMD_ISP_PAGE_ERASE:
      {
        INVALIDATE_CACHE;
        int addr = (rx_buf[9] << 8) | rx_buf[8];
        DEBUG_PRINT("CMD_ISP_PAGE_ERASE (addr: %d)\n", addr);
        N51ICP_page_erase(addr & PAGE_MASK);
        send_pkt();
      } break;
      case CMD_RUN_APROM:
      case CMD_RUN_LDROM:
      case CMD_RESET:{
        DEBUG_PRINT("exiting from ICP and running aprom...\n");
        INVALIDATE_CACHE;
        send_pkt();
        reset_conn();
      } break;
      case CMD_READ_ROM:
        dump_addr = (rx_buf[9] << 8) | rx_buf[8];
        dump_size = (rx_buf[13] << 8) | rx_buf[12];
        DEBUG_PRINT("CMD_READ_ROM (addr: %d, size: %d) \n", dump_addr, dump_size);
        start_dump(dump_addr, dump_size);
        break;

      case CMD_UPDATE_WHOLE_ROM:
        g_update_checksum = 0;
        DEBUG_PRINT("CMD_UPDATE_WHOLE_ROM\n");
        INVALIDATE_CACHE;
        update_addr = (rx_buf[9] << 8) | rx_buf[8];
        update_size = (rx_buf[13] << 8) | rx_buf[12];
        if (!is_whole_rom_update_valid((uint32_t)update_addr, update_size)) {
          fail_pkt();
          break;
        }
        // preserved_ldrom_sz = 0;
        if (!mass_erase_checked(true)) break;
        DEBUG_PRINT("flashing %d bytes\n", update_size);
        update(&rx_buf[16], 48);
        add_g_total_checksum();
        if (update_size > 0)
          state = UPDATING_STATE;
        send_pkt();
        break;

      case CMD_UPDATE_APROM: {
        g_update_checksum = 0;
        update_addr = (rx_buf[9] << 8) | rx_buf[8];
        update_size = (rx_buf[13] << 8) | rx_buf[12];
        DEBUG_PRINT("CMD_UPDATE_APROM (addr: %d, size: %d)\n", update_addr, update_size);
        if (!is_aprom_range_valid((uint32_t)update_addr, update_size)) {
          fail_pkt();
          break;
        }
        read_config(&flags);
        
        cid = N51ICP_read_cid();
        INVALIDATE_CACHE;
        // Specification states that we need to erase the aprom when we receive this command
        if (flags.LOCK != 0 && cid != 0xFF && cid != 0x00) {
          // device is not locked, we need to erase only the areas we're going to write to
          uint32_t start_addr = ((uint32_t)update_addr) & PAGE_MASK;
          uint32_t end_addr = (((uint32_t)update_addr) + update_size - 1) & PAGE_MASK;
          for (uint32_t curr_addr = start_addr; curr_addr <= end_addr; curr_addr += PAGE_SIZE){
            N51ICP_page_erase(curr_addr);
          }
        } else { // device is locked, we'll need to do a mass erase
          if (!mass_erase_checked(true)) break;
        }
        read_config(&flags);
        DEBUG_PRINT("flashing %d bytes\n", update_size);
        update(&rx_buf[16], 48);
        add_g_total_checksum();
        if (update_size > 0)
          state = UPDATING_STATE;
        send_pkt();
      } break;
      default:
        DEBUG_PRINT("unknown command 0x%02x\n", cmd);
        fail_pkt();
        break;
    }
  } else if (rx_bufhead > 0 && rx_bufhead < PACKSIZE && check_packet_timeout()){
    DEBUG_PRINT("PCKSIZE_TIMEOUT\n");
    reset_buf(); // reset the buffer
  }
#if CONNECTION_TIMEOUT
  else { // serial has no characters
    if (state > WAITING_FOR_CONNECT_CMD && curr_time - last_read_time > CONNECTION_TIMEOUT) { // 10 seconds between packets
        DEBUG_PRINT("Connection timeout, resetting...\n");
        reset_conn();
        reset_buf();
    }
  }
#endif

  service_debug_passthrough();
}
