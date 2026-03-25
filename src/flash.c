/*
 * Flash IAP — Non-volatile parameter storage
 *
 * Uses the MS51FB9AE IAP (In-Application Programming) to store
 * configuration parameters in the last page of APROM.
 *
 * APROM size = 16KB = 0x4000
 * Page size  = 128 bytes
 * Last page  = 0x3F80 - 0x3FFF
 *
 * We store the params_t structure at the start of the last page.
 * The page must be erased before writing (all bits set to 1).
 */

#include "ms51_reg.h"
#include "ms51_config.h"
#include "flash.h"

#define PARAM_PAGE_ADDR  0x3F80  /* Last 128-byte page of 16KB APROM */

static void iap_byte_read(uint16_t addr, uint8_t *data)
{
    IAPAL = (uint8_t)(addr & 0xFF);
    IAPAH = (uint8_t)(addr >> 8);
    IAPCN = IAP_BYTE_READ;
    IAPTRG = 0x5A;  /* Trigger IAP */
    /* One NOP required after trigger */
    __asm__("nop");
    *data = IAPFD;
}

static void iap_byte_program(uint16_t addr, uint8_t data)
{
    IAPAL = (uint8_t)(addr & 0xFF);
    IAPAH = (uint8_t)(addr >> 8);
    IAPFD = data;
    IAPCN = IAP_BYTE_PROGRAM;

    /* Enable IAP write (timed access required) */
    TIMED_ACCESS();
    IAPUEN = 0x01;  /* Enable APROM write */

    IAPTRG = 0x5A;
    __asm__("nop");

    TIMED_ACCESS();
    IAPUEN = 0x00;  /* Disable APROM write */
}

static void iap_page_erase(uint16_t addr)
{
    IAPAL = (uint8_t)(addr & 0xFF);
    IAPAH = (uint8_t)(addr >> 8);
    IAPCN = IAP_PAGE_ERASE;

    TIMED_ACCESS();
    IAPUEN = 0x01;

    IAPTRG = 0x5A;
    __asm__("nop");

    TIMED_ACCESS();
    IAPUEN = 0x00;
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

    for (i = 0; i < sizeof(params_t); i++) {
        iap_byte_read(PARAM_PAGE_ADDR + i, &dst[i]);
    }

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

    /* Erase the parameter page */
    iap_page_erase(PARAM_PAGE_ADDR);

    /* Write byte by byte */
    src = (const uint8_t *)&tmp;
    for (i = 0; i < sizeof(params_t); i++) {
        iap_byte_program(PARAM_PAGE_ADDR + i, src[i]);
    }
}

void flash_default_params(params_t *p)
{
    p->magic        = PARAMS_MAGIC;
    p->mode         = MODE_OPEN_LOOP;
    p->torque_limit = CURRENT_LIMIT_MA;
    p->pid_pos_kp   = PID_POS_KP;
    p->pid_pos_ki   = PID_POS_KI;
    p->pid_pos_kd   = PID_POS_KD;
    p->pid_vel_kp   = PID_VEL_KP;
    p->pid_vel_ki   = PID_VEL_KI;
    p->pid_vel_kd   = PID_VEL_KD;
    p->encoder_cpr  = ENCODER_CPR;
    p->checksum     = compute_checksum(p);
}
