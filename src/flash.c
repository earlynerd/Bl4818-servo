/*
 * Flash IAP — Non-volatile parameter storage
 *
 * Uses the MS51FB9AE IAP (In-Application Programming) to store
 * configuration parameters in the last page of the configured APROM span.
 *
 * APROM size is provided by the build configuration.
 * Page size  = 128 bytes
 * Param page = APROM_SIZE - 128
 *
 * We store the params_t structure at the start of the last page.
 * The page must be erased before writing (all bits set to 1).
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "flash.h"

/*
 * IAP helper: enable IAP mode (IAPEN in CHPCON, TA-protected).
 * Must be called before any IAP operation.
 */
static void iap_enable(void)
{
    TIMED_ACCESS();
    CHPCON |= 0x01;     /* IAPEN = 1 */
}

/*
 * IAP helper: disable IAP mode.
 * Should be called after IAP operations to save power (stops HIRC if external clock).
 */
static void iap_disable(void)
{
    TIMED_ACCESS();
    CHPCON &= ~0x01;    /* IAPEN = 0 */
}

/*
 * IAP helper: trigger IAP operation (IAPTRG is TA-protected).
 * Per TRM page 196: set IAPGO (IAPTRG.0) with timed access.
 * Interrupts must be disabled around the TA + trigger sequence.
 */
static void iap_trigger(void)
{
    uint8_t ea_save = EA;
    EA = 0;              /* Disable interrupts during IAP trigger */
    TIMED_ACCESS();
    IAPTRG |= 0x01;     /* Set IAPGO to start IAP operation */
    __asm__("nop");
    EA = ea_save;        /* Restore interrupt state */
}

static void iap_byte_read(uint16_t addr, uint8_t *data)
{
    IAPAL = (uint8_t)(addr & 0xFF);
    IAPAH = (uint8_t)(addr >> 8);
    IAPCN = IAP_BYTE_READ;
    iap_trigger();
    *data = IAPFD;
}

static void iap_byte_program(uint16_t addr, uint8_t data)
{
    IAPAL = (uint8_t)(addr & 0xFF);
    IAPAH = (uint8_t)(addr >> 8);
    IAPFD = data;
    IAPCN = IAP_BYTE_PROGRAM;

    TIMED_ACCESS();
    IAPUEN = 0x01;       /* Enable APROM write */

    iap_trigger();

    TIMED_ACCESS();
    IAPUEN = 0x00;       /* Disable APROM write */
}

static void iap_page_erase(uint16_t addr)
{
    IAPAL = (uint8_t)(addr & 0xFF);
    IAPAH = (uint8_t)(addr >> 8);
    IAPCN = IAP_PAGE_ERASE;

    TIMED_ACCESS();
    IAPUEN = 0x01;       /* Enable APROM write */

    iap_trigger();

    TIMED_ACCESS();
    IAPUEN = 0x00;       /* Disable APROM write */
}

static uint8_t compute_checksum(const params_t *p)
{
    const uint8_t *ptr = (const uint8_t *)p;
    uint8_t sum = 0;
    uint8_t i;
    /* Checksum over all fields except the checksum itself */
    for (i = 0; i < sizeof(params_t) - 1; i++) {
        sum += ptr[i];
    }
    return ~sum;  /* One's complement */
}

void flash_load_params(params_t *p)
{
    uint8_t *dst = (uint8_t *)p;
    uint8_t i;

    iap_enable();

    for (i = 0; i < sizeof(params_t); i++) {
        iap_byte_read(PARAM_PAGE_ADDR + i, &dst[i]);
    }

    iap_disable();

    /* Validate */
    if (p->magic != PARAMS_MAGIC || compute_checksum(p) != p->checksum) {
        flash_default_params(p);
    }
}

void flash_save_params(const params_t *p)
{
    params_t tmp;
    const uint8_t *src;
    uint8_t i;

    /* Copy and compute checksum */
    tmp = *p;
    tmp.magic = PARAMS_MAGIC;
    tmp.checksum = compute_checksum(&tmp);

    iap_enable();

    /* Erase the parameter page */
    iap_page_erase(PARAM_PAGE_ADDR);

    /* Write byte by byte */
    src = (const uint8_t *)&tmp;
    for (i = 0; i < sizeof(params_t); i++) {
        iap_byte_program(PARAM_PAGE_ADDR + i, src[i]);
    }

    iap_disable();
}

void flash_default_params(params_t *p)
{
    p->magic        = PARAMS_MAGIC;
    p->mode         = MODE_OPEN_LOOP;
    p->torque_limit = DEFAULT_TORQUE_LIMIT_MA;
    p->pid_pos_kp   = PID_POS_KP;
    p->pid_pos_ki   = PID_POS_KI;
    p->pid_pos_kd   = PID_POS_KD;
    p->pid_vel_kp   = PID_VEL_KP;
    p->pid_vel_ki   = PID_VEL_KI;
    p->pid_vel_kd   = PID_VEL_KD;
    p->encoder_cpr  = ENCODER_CPR;
    p->commutation_offset = DEFAULT_COMMUTATION_OFFSET;
    p->checksum     = compute_checksum(p);
}
