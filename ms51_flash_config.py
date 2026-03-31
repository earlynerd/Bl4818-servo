"""
Checked-in MS51 config profiles for BL4818-servo.

`flash.py --write-config` uses `PROJECT_DEFAULT_PROFILE`.
`flash.py --recover` uses `PROJECT_RECOVERY_PROFILE`.

Update this file when the project's preferred target config changes so new
checkouts inherit the right settings without anyone needing to remember raw
hex bytes from a previous session.

Config byte order is:
    [CONFIG0, CONFIG1, CONFIG2, CONFIG3, CONFIG4]

-------------CONFIG byte cheatsheet--------------
CONFIG0: 0bX1XX1XX1
    [7] CBS CONFIG Boot Select: This bit defines from which block that MCU re-boots after resets except software reset.
            1 = MCU will re-boot from APROM after resets except software reset.
            0 = MCU will re-boot from LDROM after resets except software reset.
    [5] OCDPWM PWM Output State Under OCD Halt: This bit decides the output state of PWM when OCD halts CPU.
            1 = Tri-state pins those are used as PWM outputs.
            0 = PWM continues.
            Note that this bit is valid only when the corresponding PIO bit of PWM channel is set as 1.
    [4] OCDEN OCD Enable
            1 = OCD Disabled.
            0 = OCD Enabled.
            Note: If MCU run in OCD debug mode and OCDEN = 0, hard fault reset will be disabled and
            only Hard F flag be asserted.
    [2] RPD Reset Pin Disable
            1 = The reset function of P2.0/Nrst pin Enabled. P2.0/Nrst functions as the external reset pin.
            0 = The reset function of P2.0/Nrst pin Disabled. P2.0/Nrst functions as an input-only pin P2.0.
    [1] LOCK Chip Lock Enable
            1 = Chip is unlocked. Flash Memory is not locked. Their contents can be read out through a
            parallel Writer/ICP programmer.
            0 = Chip is locked. Whole Flash Memory is locked. Their contents read through a parallel
            Writer or ICP programmer will be all blank (FFH)
            
CONFIG1: 0b11111XXX
    [2:0] LDSIZE[2:0] LDROM Size Select
            111 = No LDROM. APROM is 16 Kbytes.
            110 = LDROM is 1 Kbytes. APROM is 15 Kbytes.
            101 = LDROM is 2 Kbytes. APROM is 14 Kbytes.
            100 = LDROM is 3 Kbytes. APROM is 13 Kbytes.
            0xx = LDROM is 4 Kbytes. APROM is 12 Kbytes.
            
CONFIG2: 0bX1XXXX1
    [7] CBODEN CONFIG Brown-Out Detect Enable
            1 = Brown-out detection circuit on.
            0 = Brown-out detection circuit off.
    [5:4] CBOV[1:0] CONFIG Brown-Out Voltage Select
            11 = VBOD is 2.2V.
            10 = VBOD is 2.7V.
            01 = VBOD is 3.7V.
            00 = VBOD is 4.4V.
    [3] BOIAP Brown-Out Inhibiting IAP: This bit decides whether IAP erasing or programming is inhibited by brown-out status. This bit is valid only when brown-out detection is enabled.
            1 = IAP erasing or programming is inhibited if VDD is lower than VBOD.
            0 = IAP erasing or programming is allowed under any workable VDD.
    [2] CBORST CONFIG Brown-Out Reset Enable: This bit decides whether a brown-out reset is caused by a power drop below VBOD.
            1 = Brown-out reset Enabled.
            0 = Brown-out reset Disabled
            
CONFIG3: mysteriously there is no CONFIG3. move along please. 0b11111111

CONFIG4: 0bXXXX1111
    [7:4] WDTEN[3:0] WDT Enable: This field configures the WDT behavior after MCU execution.
            1111 = WDT is Disabled. WDT can be used as a general purpose timer via software control.
            0101 = WDT is Enabled as a time-out reset timer and it stops running during Idle or Powerdown mode.
            Others = WDT is Enabled as a time-out reset timer and it keeps running during Idle or Powerdown mode.
"""

PROJECT_DEFAULT_PROFILE = "servo-bor-3v7-wdt"
PROJECT_RECOVERY_PROFILE = "factory-safe"

CONFIG_PROFILES = {
    "factory-safe": {
        "bytes": bytes([0xFF, 0xFF, 0xFF, 0xFF, 0xFF]),
        "description": (
            "APROM boot, reset pin enabled, unlocked, no LDROM, "
            "BOD/BOR enabled at 2.2V, WDT software-controlled"
        ),
    },
    "servo-bor-3v7": {
        "bytes": bytes([0xFF, 0xFF, 0xDF, 0xFF, 0xFF]),
        "description": (
            "Same safe baseline, but raises BOD/BOR threshold to 3.7V; "
            "WDT remains software-controlled"
        ),
    },
    "servo-bor-3v7-wdt": {
        "bytes": bytes([0xFF, 0xFF, 0xDF, 0xFF, 0x5F]),
        "description": (
            "Same as servo-bor-3v7, but CONFIG4 also enables WDT reset "
            "and stops it in idle/power-down"
        ),
    },
}
