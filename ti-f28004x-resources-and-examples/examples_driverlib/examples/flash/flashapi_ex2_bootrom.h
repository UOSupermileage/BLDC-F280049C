//###########################################################################
//
// FILE:   flashapi_ex2_bootrom.h
//
// TITLE:  BootROM Definitions.
//
//###########################################################################
// $TI Release: F28004x Support Library v1.10.00.00 $
// $Release Date: Tue May 26 17:06:03 IST 2020 $
// $Copyright:
// Copyright (C) 2020 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################



#ifndef __FLASHAPI_EX2_BOOTROM_H__
#define __FLASHAPI_EX2_BOOTROM_H__

#include <stdint.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_sci.h"
#include "inc/hw_i2c.h"
#include "sysctl.h"
#include "cpu.h"
#include "interrupt.h"
#include "flash.h"
#include "dcsm.h"
#include "pin_map.h"
#include "gpio.h"
#include "sci.h"
#include "spi.h"
#include "i2c.h"
#include "can.h"

//When the ROM is run with an emulator connected, these four addresses are used
//to emulate OTP configuration.
#define EMU_BOOTPIN_CONFIG  0xD00    //Equivalent to GPREG1
#define EMU_BOOT_GPREG2     0xD02    //Equivalent to GPREG2
#define EMU_BOOTDEF_LOW     0xD04    //Equivalent to GPREG3
#define EMU_BOOTDEF_HIGH    0xD06    //Equivalent to BOOTCTRL

//Emulation boot pin configuration fields. Currently, only EMU_BOOTPIN_CONFIG_KEY
//is used.
#define EMU_BOOTPIN_CONFIG_KEY      ((HWREG(EMU_BOOTPIN_CONFIG) & (uint32_t)0xFF000000) >> 24)
//#define EMU_BOOTPIN_CONFIG_BMSP2  ((HWREG(EMU_BOOTPIN_CONFIG) & (uint32_t)0x00FF0000) >> 16)
//#define EMU_BOOTPIN_CONFIG_BMSP1  ((HWREG(EMU_BOOTPIN_CONFIG) & (uint32_t)0x0000FF00) >> 8)
//#define EMU_BOOTPIN_CONFIG_BMSP0  ((HWREG(EMU_BOOTPIN_CONFIG) & (uint32_t)0x000000FF))

#define EMUBOOTDEF_L(x)             ((HWREG(EMU_BOOTDEF_LOW)  & ((uint32_t)0xFF << (8*x))) >> (8*x))
#define EMUBOOTDEF_H(x)             ((HWREG(EMU_BOOTDEF_HIGH) & ((uint32_t)0xFF << (8*x))) >> (8*x))

#define Z1_OTP_BOOTPIN_CONFIG       (DCSMBANK0_Z1_BASE + DCSM_O_B0_Z1_GPREG1)
#define Z1_OTP_BOOT_GPREG2          (DCSMBANK0_Z1_BASE + DCSM_O_B0_Z1_GPREG2)
#define Z1_OTP_BOOTDEF_LOW          (DCSMBANK0_Z1_BASE + DCSM_O_B0_Z1_GPREG3)
#define Z1_OTP_BOOTDEF_HIGH         (DCSMBANK0_Z1_BASE + DCSM_O_B0_Z1_BOOTCTRL)

/*
Z1-GPREG2[31:24] => VALIDITY_KEY (=0x5A);
Z1-GPREG2[23:8] => RESERVED; no usage defined yet.
Z1-GPREG2[7:6] => 00 -> RUN PBIST (includes checksum test on 128KB unsecure ROM)
                  01 – Reserved (will not run PBIST)
                  10 - Reserved (will not run PBIST)
                  11 – Reserved (will not run PBIST)
Z1-GPREG2[5:4] => ERROR_STS_PIN config; this tells which GPIO pin is supposed to be used as ERROR_PIN and boot ROM configures the mux as such for the said pin.
                0 – GPIO24, MUX Option 13
                1 – GPIO28, MUX Option 13
                2 – GPIO29, MUX Option 13
                3 – ERROR_STS function Disable  (default)

Z1-GPREG2[3:0]  =>  CJTAGNODEID[3:0];
                boot ROM takes this values and programs the lower 4 bits of the CJTAGNODEID register.
*/

#define Z1_OTP_BOOT_GPREG2_KEY            ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0xFF000000) >> 24)
#define Z1_OTP_BOOT_GPREG2_PBIST_CONFIG   ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0x000000C0) >> 6)
#define Z1_OTP_BOOT_GPREG2_ERRSTS_CONFIG  ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0x00000030) >> 4)
#define Z1_OTP_BOOT_GPREG2_CJTAGNODEID    ((HWREG(Z1_OTP_BOOT_GPREG2) & (uint32_t)0x0000000F))

#define Z1_OTP_BOOTPIN_CONFIG_KEY       ((HWREG(Z1_OTP_BOOTPIN_CONFIG) & (uint32_t)0xFF000000) >> 24)
//#define Z1_OTP_BOOTPIN_CONFIG_BMSP0   ((HWREG(Z1_OTP_BOOTPIN_CONFIG) & (uint32_t)0x000000FF))
//#define Z1_OTP_BOOTPIN_CONFIG_BMSP1   ((HWREG(Z1_OTP_BOOTPIN_CONFIG) & (uint32_t)0x0000FF00) >> 8)
//#define Z1_OTP_BOOTPIN_CONFIG_BMSP2   ((HWREG(Z1_OTP_BOOTPIN_CONFIG) & (uint32_t)0x00FF0000) >> 16)

#define Z1_OTP_BOOTDEF_L(x)             ((HWREG(Z1_OTP_BOOTDEF_LOW)  & ((uint32_t)0xFF << (8*x))) >> (8*x))
#define Z1_OTP_BOOTDEF_H(x)             ((HWREG(Z1_OTP_BOOTDEF_HIGH) & ((uint32_t)0xFF << (8*x))) >> (8*x))

//#define EMU_BOOTDEF0              ((HWREG(EMU_BOOTDEF_LOW) & 0xFF) & 0x1F)
//#define EMU_BOOTDEF0_ALT_OPTIONS  (((HWREG(EMU_BOOTDEF_LOW) & 0xFF) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF1              (((HWREG(EMU_BOOTDEF_LOW) & 0xFF00)>>8) & 0x1F)
//#define EMU_BOOTDEF1_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_LOW) & 0xFF00)>>8) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF2              (((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF0000) >> 16) & 0x1F)
//#define EMU_BOOTDEF2_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF0000) >> 16) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF3              (((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF000000) >> 24) & 0x1F)
//#define EMU_BOOTDEF3_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_LOW) & (uint32_t)0xFF000000) >> 24) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF4              ((HWREG(EMU_BOOTDEF_HIGH) & 0xFF) & 0x1F)
//#define EMU_BOOTDEF4_ALT_OPTIONS  (((HWREG(EMU_BOOTDEF_HIGH) & 0xFF) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF5              (((HWREG(EMU_BOOTDEF_HIGH) & 0xFF00)>>8) & 0x1F)
//#define EMU_BOOTDEF5_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_HIGH) & 0xFF00)>>8) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF6              (((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF0000) >> 16) & 0x1F)
//#define EMU_BOOTDEF6_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF0000) >> 16) & 0xE0) >> 5)
//
//#define EMU_BOOTDEF7              (((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF000000) >> 24) & 0x1F)
//#define EMU_BOOTDEF7_ALT_OPTIONS  ((((HWREG(EMU_BOOTDEF_HIGH) & (uint32_t)0xFF000000) >> 24) & 0xE0) >> 5)


#define FACTORY_DEFAULT_BMSP0       32      //GPIO32
#define FACTORY_DEFAULT_BMSP1       24      //GPIO24

#define PARALLEL_BOOT           0x00

#define SCI_BOOT                0x01    //GPIO28; //GPIO29 (CCARD)
#define SCI_BOOT_ALT1           0x21    //GPIO16; //GPIO17
#define SCI_BOOT_ALT2           0x41    //GPIO8; GPIO9
//#define SCI_BOOT_ALT3           0x61    //GPIO48, GPIO49
#define SCI_BOOT_ALT4           0x81    //GPIO24, GPIO25
//#define SCI_BOOT_ALT4         0x81
//#define SCI_BOOT_ALT5         0xA1
//#define SCI_BOOT_ALT6         0xC1
//#define SCI_BOOT_ALT7         0xE1


#define CAN_BOOT                0x02    //GPIO32; GPIO33
#define CAN_BOOT_ALT1           0x22    //GPIO4; GPIO5
#define CAN_BOOT_ALT2           0x42    //GPIO30; GPIO31
#define CAN_BOOT_ALT3           0x62    //GPIO35; GPIO37
#define CAN_BOOT_SENDTEST       0x82    //GPIO32; GPIO33
#define CAN_BOOT_ALT1_SENDTEST  0xA2    //GPIO4; GPIO5
#define CAN_BOOT_ALT2_SENDTEST  0xC2    //GPIO30; GPIO31
#define CAN_BOOT_ALT3_SENDTEST  0xE2    //GPIO35; GPIO37

#define FLASH_BOOT              0x03    //begin of BANK 0 Sector 0
#define FLASH_BOOT_ALT1         0x23    //begin of BANK 0 Sector 15
#define FLASH_BOOT_ALT2         0x43    //begin of BANK 1 Sector 0
#define FLASH_BOOT_ALT3         0x63    //begin of BANK 1 Sector 15

#define WAIT_BOOT               0x04    //with WDOG enabled
#define WAIT_BOOT_ALT1          0x24    //without WDOG enabled

#define RAM_BOOT                0x05
//#define RAM_BOOT_ALT1         0x25
//#define RAM_BOOT_ALT2         0x45
//#define RAM_BOOT_ALT3         0x65


#define SPI_MASTER_BOOT         0x06    //GPIO16-GPIO19
#define SPI_MASTER_BOOT_ALT1    0x26    //GPIO8-GPIO11
#define SPI_MASTER_BOOT_ALT2    0x46    //GPIO54-GPIO57
#define SPI_MASTER_BOOT_ALT3    0x66    //GPIO16, GPIO17, GPIO56, GPIO57
#define SPI_MASTER_BOOT_ALT4    0x86    //GPIO8, GPIO17, GPIO9, GPIO11


#define I2C_MASTER_BOOT         0x07    //GPIO32, GPIO33
#define I2C_MASTER_BOOT_ALT1    0x27    //GPIO18; GPIO19
#define I2C_MASTER_BOOT_ALT2    0x47    //GPIO26; GPIO27
#define I2C_MASTER_BOOT_ALT3    0x67    //GPIO42; GPIO43

#define PLC_BOOT                0x08

#define BROM_ASYSCTL_O_PMMREFTRIM   0x12U       //Power Management Module Reference Trim Register
#define BROM_ASYSCTL_O_PMMVREGTRIM  0x14U       //Power Management Module VREG Trim Register
#define BROM_ASYSCTL_O_PMMVMONTRIM  0x0EU       //Power Management Module VMON Trim Register
#define BROM_ASYSCTL_O_PMMVMONTRIM2 0x10U       //Power Management Module VMON Trim Register

#define BROM_ASYSCTL_O_OSCREFTRIM   0x06U       // OSC IREF,BGSLOPE,BGVAL TRIM
#define BROM_ASYSCTL_O_OSC1_TRIM    0x00U       // INTOSC1 SLOPE, VALFINE TRIMs
#define BROM_ASYSCTL_O_OSC2_TRIM    0x02U       // INTOSC2 SLOPE, VALFINE TRIMs


//---------------------------------------------------------------------------
// Fixed boot entry points:
//
#define FLASH_ENTRY_POINT               0x080000    //BANK0 Sector 0
#define FLASH_ENTRY_POINT_ALT1          0x08EFF0    //BANK0 Sector 15 end - 16
#define FLASH_ENTRY_POINT_ALT2          0x090000    //BANK1 sector 0
#define FLASH_ENTRY_POINT_ALT3          0x09EFF0    //BANK1 sector 15 end - 16


#define RAM_ENTRY_POINT                 0x000000    //M0 start address

#define PLC_ENTRY_POINT_IN_ROM          0x3F1800    //APPBOOT

#define ERROR                   1
#define NO_ERROR                0

#define BROM_EIGHT_BIT_HEADER           0x08AA

//---------------------------------------------------------------------------
//
#define TI_OTP_PARTID_L                 (*(volatile uint32_t *)(0x70200))
#define TI_OTP_PARTID_H                 (*(volatile uint32_t *)(0x70202))


#define TI_OTP_DCX_REG_BASE_ADDRESS     0x70204
#define TI_OTP_REG_DC00                 (*(volatile uint32_t *)(0x70204))
#define TI_OTP_REG_DC01                 (*(volatile uint32_t *)(0x70206))
#define TI_OTP_REG_DC02                 (*(volatile uint32_t *)(0x70208))
#define TI_OTP_REG_DC03                 (*(volatile uint32_t *)(0x7020A))
#define TI_OTP_REG_DC04                 (*(volatile uint32_t *)(0x7020C))
#define TI_OTP_REG_DC05                 (*(volatile uint32_t *)(0x7020E))
#define TI_OTP_REG_DC06                 (*(volatile uint32_t *)(0x70210))
#define TI_OTP_REG_DC07                 (*(volatile uint32_t *)(0x70212))
#define TI_OTP_REG_DC08                 (*(volatile uint32_t *)(0x70214))
#define TI_OTP_REG_DC09                 (*(volatile uint32_t *)(0x70216))
#define TI_OTP_REG_DC10                 (*(volatile uint32_t *)(0x70218))
#define TI_OTP_REG_DC11                 (*(volatile uint32_t *)(0x7021A))
#define TI_OTP_REG_DC12                 (*(volatile uint32_t *)(0x7021C))
#define TI_OTP_REG_DC13                 (*(volatile uint32_t *)(0x7021E))
#define TI_OTP_REG_DC14                 (*(volatile uint32_t *)(0x70220))
#define TI_OTP_REG_DC15                 (*(volatile uint32_t *)(0x70222))
#define TI_OTP_REG_DC16                 (*(volatile uint32_t *)(0x70224))
#define TI_OTP_REG_DC17                 (*(volatile uint32_t *)(0x70226))
#define TI_OTP_REG_DC18                 (*(volatile uint32_t *)(0x70228))
#define TI_OTP_REG_DC19                 (*(volatile uint32_t *)(0x7022A))
#define TI_OTP_REG_DC20                 (*(volatile uint32_t *)(0x7022C))
#define TI_OTP_REG_PERCNF1              (*(volatile uint32_t *)(0x7022E))
#define TI_OTP_REG_DC21                 (*(volatile uint32_t *)(0x70230))
#define TI_OTP_REG_DC22                 (*(volatile uint32_t *)(0x70232))
#define TI_OTP_REG_DC23                 (*(volatile uint32_t *)(0x70234))
#define TI_OTP_REG_DC24                 (*(volatile uint32_t *)(0x70236))
#define TI_OTP_REG_DC25                 (*(volatile uint32_t *)(0x70238))

#define TI_OTP_REG_CJTAGNODEID          (*(volatile uint32_t *)(0x7026A))
#define TI_OTP_REG_VREGCTL_ENMASK       (*(volatile uint16_t *)(0x7026C))

#define TI_OTP_CPUROM_DC1               (*(volatile uint32_t *)(0x70604))
#define TI_OTP_CPUROM_DC2               (*(volatile uint32_t *)(0x70606))
#define TI_OTP_CPUROM_DC3               (*(volatile uint32_t *)(0x70608))
#define TI_OTP_CPUROM_DC4               (*(volatile uint32_t *)(0x7060A))

#define TI_OTP_CLAROM_DC1               (*(volatile uint32_t *)(0x70624))
#define TI_OTP_CLAROM_DC2               (*(volatile uint32_t *)(0x70626))
#define TI_OTP_CLAROM_DC3               (*(volatile uint32_t *)(0x70628))
#define TI_OTP_CLAROM_DC4               (*(volatile uint32_t *)(0x7062A))

#define TI_OTP_PKG_TYPE                 (*(volatile uint32_t *)(0x7064C))


#define BROM_SYSCTL_O_CPUROM_DC1                0x140U
#define BROM_SYSCTL_O_CPUROM_DC2                0x142U
#define BROM_SYSCTL_O_CPUROM_DC3                0x144U
#define BROM_SYSCTL_O_CPUROM_DC4                0x146U

#define BROM_SYSCTL_O_CLAROM_DC1                0x160U
#define BROM_SYSCTL_O_CLAROM_DC2                0x162U
#define BROM_SYSCTL_O_CLAROM_DC3                0x164U
#define BROM_SYSCTL_O_CLAROM_DC4                0x166U

#define BROM_SYSCTL_O_PKG_TYPE                  0x12EU

#define BROM_ANALOG_SYSCTL_O_VREGCTL            0x006A

//bits15:8 is the KEY ; if Value == 0x5A then the remaining bits are valid
//bits 7:2 => reserved
//bits 0:1 if set to b'00 BROM will program 0x01 in VREGCTL.ENMASK
//          - any other value the VREGCTL.ENMASK will be left at reset state.

#define TI_OTP_REG_VREGCTL_ENMASK_VAL   ((TI_OTP_REG_VREGCTL_ENMASK) & 0x03)

#define TI_OTP_REG_VREGCTL_ENMASK_KEY   (((TI_OTP_REG_VREGCTL_ENMASK) & 0xFF00) >> 0x8)


#define TI_OTP_SECDC                    0x703F0

//---------------------------------------------------------------------------
//


typedef uint16_t (*uint16fptr)();
extern  uint16fptr GetWordData;

#define DEVICE_CAL_LOCATION   0x70280
#define CBROM_DEVCAL          ((void (*)(void))DEVICE_CAL_LOCATION)

#define OTP_VERSION_FOR_BOOT    (*(volatile uint16_t *)(0x7026D))

//Bits [1:0]    If 01, enable the PLL, otherwise leave it disabled
//Bits [7:2]    PLL divider to use when the PLL is enabled
//Bits [31:24]  If 0x5A, use this configuration word, otherwise use the default settings
#define OTP_BOOT_CONFIGURE_WORD         (*(volatile uint32_t *)0x703EE)


#define OTP_BOOT_ESCAPE_TABLE_END       0x703EC

//extern void otp_func_refs();
#define TI_OTP_C1BROM_ESCAPE_POINT_15           (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-28))
#define TI_OTP_C1BROM_ESCAPE_POINT_14           (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-26))
#define TI_OTP_C1BROM_ESCAPE_POINT_13           (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-24))
#define TI_OTP_C1BROM_ESCAPE_POINT_12           (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-22))
#define TI_OTP_C1BROM_ESCAPE_POINT_11           (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-20))
#define TI_OTP_C1BROM_ESCAPE_POINT_10           (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-18))
#define TI_OTP_C1BROM_ESCAPE_POINT_9            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-16))
#define TI_OTP_C1BROM_ESCAPE_POINT_8            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-14))
#define TI_OTP_C1BROM_ESCAPE_POINT_7            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-12))
#define TI_OTP_C1BROM_ESCAPE_POINT_6            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-10))
#define TI_OTP_C1BROM_ESCAPE_POINT_5            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-8))
#define TI_OTP_C1BROM_ESCAPE_POINT_4            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-6))
#define TI_OTP_C1BROM_ESCAPE_POINT_3            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-4))
#define TI_OTP_C1BROM_ESCAPE_POINT_2            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-2))
#define TI_OTP_C1BROM_ESCAPE_POINT_1            (*(volatile uint32_t *)((OTP_BOOT_ESCAPE_TABLE_END)-0))


#define TI_OTP_PMM_LC_TRIM_KEY                  (*(volatile uint16_t *)0x70274)
#define TI_OTP_PMM_LC_BGSLOPE_VAL_IREF_TRIM     (*(volatile uint16_t *)0x70270)
#define TI_OTP_PMM_LC_VMON_TRIM                 (*(volatile uint16_t *)0x70271)
#define TI_OTP_PMM_LC_VMON_TRIM_2               (*(volatile uint16_t *)0x70272)
#define TI_OTP_PMM_LC_VREG_TRIM                 (*(volatile uint16_t *)0x70273)

#define TI_OTP_OSC_TRIM_KEY                     (*(volatile uint16_t *)0x70275)
#define TI_OTP_OSC1_TRIM                        (*(volatile uint32_t *)0x70276)
#define TI_OTP_OSC2_TRIM                        (*(volatile uint32_t *)0x70278)
#define TI_OTP_OSC_REF_TRIM                     (*(volatile uint16_t *)0x7027A)


#define BROM_DCSM_O_Zx_EXEONLYRAM       0x0
#define BROM_DCSM_O_Zx_EXEONLYSECT      0x2
#define BROM_DCSM_O_Zx_GRABRAM          0x4
#define BROM_DCSM_O_Zx_GRABSECT         0x6

//boot status information
#define BOOTROM_START_BOOT             0x00000001  //Set during the initialization phase of the boot ROM
#define BOOTROM_IN_FLASH_BOOT          0x00000002
#define BOOTROM_IN_PARALLEL_BOOT       0x00000004
#define BOOTROM_IN_RAM_BOOT            0x00000008
#define BOOTROM_IN_SCI_BOOT            0x00000010
#define BOOTROM_IN_SPI_BOOT            0x00000020
#define BOOTROM_IN_I2C_BOOT            0x00000040
#define BOOTROM_IN_CAN_BOOT            0x00000080
#define BOOTROM_IN_PLC_BOOT            0x00000100

#define BOOTROM_DCSM_INIT_DONE         0x00000200
#define BOOTROM_POR_MEM_TEST_DONE      0x00000400
#define BOOTROM_RESC_HANDLED           0x00000800
#define BOOTROM_HANDLED_XRSN           0x00001000
#define BOOTROM_HANDLED_POR            0x00002000
#define BOOTROM_BOOT_COMPLETE          0x00008000


#define BOOTROM_IN_AN_ITRAP            0x00010000
#define BOOTROM_GOT_A_PIEMISMATCH      0x00020000
//#define BOOTROM_GOT_A_OVF_NMI          0x00040000
#define BOOTROM_GOT_A_RL_NMI           0x00080000
#define BOOTROM_GOT_A_FLASH_UNCERR_NMI 0x00100000
#define BOOTROM_GOT_A_RAM_UNCERR_NMI   0x00200000
#define BOOTROM_GOT_A_MCLK_NMI         0x00400000
//#define BOOTROM_GOT_A_DCDCOLF_NMI      0x00800000

#define BOOTROM_SYSCLK                 10000000U



//Function prototypes
extern void cbrom_configure_flash();
extern void InitDCSM();
extern uint32_t Gather_Bx_Zx_ZSB(uint16_t bank, uint16_t zone, uint32_t *csmkey);

extern interrupt void cbrom_itrap_isr(void);
extern interrupt void cbrom_handle_nmi();

extern uint16_t cbrom_decode_bootpins(uint32_t pinconfig);
extern uint32_t cbrom_GPIO_ReadPin(uint32_t pin);

extern uint32_t getLongData(void);
extern void copyData(void);
extern void readReservedFn(void);

extern uint32_t sciBoot(uint32_t  BootMode);


extern uint16_t SelectBootMode(void);
extern unsigned short verify_pbist_checksum_onROM();

#endif // __FLASHAPI_EX2_BOOTROM_H__
