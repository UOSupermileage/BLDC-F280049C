//#############################################################################
//
// FILE:   erad_ex7_ctm_profile_function.c
//
// TITLE:  ERAD CTM Profile Function.
//
//! \addtogroup driver_example_list
//! <h1>ERAD CTM Profile Function</h1>
//!
//!  This example uses HWBP1, HWBP2 and COUNTER1 of the ERAD module to
//!  to profile a function. Two dummy variable are written to inside the
//!  function (delayFunction), startCounts and endCounts. The writes to these
//!  variables trigger HWBP1 and HWBP2 respectively. The COUNTER1 module is
//!  setup to operate in START-STOP mode and count the number of CPU cycles
//!  elapsed between the the two HWBPs.
//!
//!  \b Watch \b Variables \n
//!  - numberOfCPUCycles - the number of cpu cycles
//!
//! \b External \b Connections \n
//!  None
//
//#############################################################################
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
//#############################################################################

//
// Included Files
//
#include <stdint.h>
#include "driverlib.h"
#include "device.h"
#include "inc/hw_erad.h"

//
// Global Variables
//

//
// This variable is monitored by the HWBP1 from ERAD
// A write to this variable, triggers HWBP1
//
volatile uint32_t startCounts = 0;

//
// This variable is monitored by the HWBP2 from ERAD
// A write to this variable, triggers HWBP2
//
volatile uint32_t endCounts = 0;

//
// This variable will contain the number of CPU cycles
// elapsed between HWBP1 and HWBP2 to profile the
// delayFunction
//
volatile uint32_t numberOfCPUCycles = 0;

//
// Function Prototypes
//
void delayFunction(void);
void initHWBPCheckDataWriteBus(uint32_t base, uint32_t address);
void initCTMStartStopModeCPUCycleCount(uint32_t base);

//
// Main
//
void main(void)
{
    //
    // Initializes device clock and peripherals
    //
    Device_init();

    //
    // Configures the GPIO pin as a push-pull output
    //
    Device_initGPIO();

    //
    // Initializes PIE and clears PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initializes the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Application is owner
    //
    __eallow();
    HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_OWNER) = 1;
    __edis();

    //
    // Set up HWBPs
    //
    initHWBPCheckDataWriteBus(ERAD_HWBP1_BASE, (uint32_t)&startCounts);
    initHWBPCheckDataWriteBus(ERAD_HWBP2_BASE, (uint32_t)&endCounts);

    //
    // Set up CTM
    //
    initCTMStartStopModeCPUCycleCount(ERAD_COUNTER1_BASE);

    //
    // Enable HWBP
    //
    __eallow();
    HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_ENABLE) |=
            ERAD_GLBL_ENABLE_HWBP1 | ERAD_GLBL_ENABLE_HWBP2 | ERAD_GLBL_ENABLE_CTM1;
    __edis();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    delayFunction();
    numberOfCPUCycles = HWREG(ERAD_COUNTER1_BASE + ERAD_O_CTM_MAX_COUNT);

    //
    // IDLE loop. Just sit and loop forever (optional)
    //
    while(1)
    {
        //View "numberOfCPUCycles" variable in CCS
        ESTOP0;
    }
}

//
// HWBP to monitor a write to an address
//
void initHWBPCheckDataWriteBus(uint32_t base, uint32_t address)
{

    __eallow();

    //
    // Regular Masked Compare
    //
    HWREGH(base + ERAD_O_HWBP_CNTL) =
            ((HWREGH(base + ERAD_O_HWBP_CNTL) & (~ERAD_HWBP_CNTL_COMP_MODE_M))|
                    ((uint16_t)0x0 << ERAD_HWBP_CNTL_COMP_MODE_S));

    //
    // DWAB comparison for address
    //
    HWREGH(base + ERAD_O_HWBP_CNTL) =
            ((HWREGH(base + ERAD_O_HWBP_CNTL) & (~ERAD_HWBP_CNTL_BUS_SEL_M))|
                    ((uint16_t)0x2 << ERAD_HWBP_CNTL_BUS_SEL_S));

    //
    // Clear HWBP Event
    //
    HWREGH(base + ERAD_O_HWBP_CLEAR) |= ERAD_HWBP_CLEAR_EVENT_CLR;

    //
    // Do not set a mask
    //
    HWREG(base + ERAD_O_HWBP_MASK) = 0x0;

    //
    // Set Reference Address
    //
    HWREG(base + ERAD_O_HWBP_REF) = address;


    __edis();
}

//
// CTM Start Stop mode to count CPU Cycles between HWBP1 and HWBP2
//
void initCTMStartStopModeCPUCycleCount(uint32_t base)
{
    __eallow();

    //
    // Reset input select is not used
    //
    HWREGH(base + ERAD_O_CTM_CNTL) =
            ((HWREGH(base + ERAD_O_CTM_CNTL) & (~ERAD_CTM_CNTL_RST_INP_SEL_M))|
                    ((uint16_t)0x0 << ERAD_CTM_CNTL_RST_INP_SEL_S));
    //
    // Reset input select is disable
    //
    HWREGH(base + ERAD_O_CTM_CNTL) &= ~ERAD_CTM_CNTL_RST_EN;

    //
    // Count as long as event is active (not edges)
    //
    HWREGH(base + ERAD_O_CTM_CNTL) &= ~ERAD_CTM_CNTL_EVENT_MODE;

    //
    // Start Stop mode
    //
    HWREGH(base + ERAD_O_CTM_CNTL) |= ERAD_CTM_CNTL_START_STOP_MODE;

    //
    // Start input select HWBP1
    //
    HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
                ((HWREGH(base + ERAD_O_CTM_INPUT_SEL) & (~ERAD_CTM_INPUT_SEL_STA_INP_SEL_M))|
                        ((uint16_t)0x0 << ERAD_CTM_INPUT_SEL_STA_INP_SEL_S));
    //
    // Stop input select HWBP2
    //
    HWREGH(base + ERAD_O_CTM_INPUT_SEL) =
                ((HWREGH(base + ERAD_O_CTM_INPUT_SEL) & (~ERAD_CTM_INPUT_SEL_STO_INP_SEL_M))|
                        ((uint16_t)0x1 << ERAD_CTM_INPUT_SEL_STO_INP_SEL_S));

    //
    // Count CPU Cycles
    //
    HWREGH(base + ERAD_O_CTM_INPUT_SEL) &= ~ERAD_CTM_INPUT_SEL_CTM_INP_SEL_EN;

    __edis();
}

//
// delay function
//
void delayFunction(void)
{
    uint32_t delay;

    //
    // Write to startCounts trigger HWBP1 to start the ERAD COUNTER1
    //
    startCounts++;
    for (delay = 1000; delay>1; delay--)
    {
        NOP;
    }

    //
    // Write to endCounts trigger HWBP2 to stop the ERAD COUNTER1
    //
    endCounts++;
}


//
// End of File
//
