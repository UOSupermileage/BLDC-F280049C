//#############################################################################
//
// FILE:   erad_ex5_hwbp_program_counter.c
//
// TITLE:  ERAD HWBP Monitor Program Counter.
//
//! \addtogroup driver_example_list
//! <h1>ERAD HWBP Monitor Program Counter</h1>
//!
//!  This example uses HWBP1 to monitor the Program Counter. The HWBP is set
//!  to monitor the program counter and STOP (HALT) the CPU when an the
//!  function "delayFunction" is executed. An RTOS interrupt is also generated
//!  when the HWBP is triggered. Inside the RTOS interrupt, a GPIO is toggled
//!  and this can be monitored on an oscilloscope.
//!
//!  \b Watch \b Variables \n
//!  None
//!
//! \b External \b Connections \n
//!  - GPIO0 toggled when delayFunction is executed and an RTOS interrupt
//!    occurs.
//!  - GPIO1 toggled in the main loop, before and after calling
//!    the dalyFunction
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
// Function Prototypes
//
interrupt void RTOSISR(void) __attribute__((ramfunc));
void delayFunction(void);
void initHWBP1CheckProgramCounter(uint32_t address);

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
    // ISRs for each RTOS interrupt
    //
    Interrupt_register(INT_RTOS, &RTOSISR);
    Interrupt_enableInCPU(INTERRUPT_CPU_RTOSINT);

    //
    // Enable RTOS Interrupt
    //
    Interrupt_enable(INT_RTOS);

    GPIO_setPinConfig(GPIO_0_GPIO0);
    GPIO_setDirectionMode(0, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_writePin(0, 1);

    GPIO_setPinConfig(GPIO_1_GPIO1);
    GPIO_setDirectionMode(1, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(1, GPIO_PIN_TYPE_STD);
    GPIO_writePin(1, 1);

    initHWBP1CheckProgramCounter((uint32_t)&delayFunction);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // IDLE loop. Just sit and loop forever (optional)
    //
    while(1)
    {
        DEVICE_DELAY_US(1000);
        GPIO_togglePin(1);
        delayFunction();
        GPIO_togglePin(1);
    }
}

//
// HWBP to halt the device
//
void initHWBP1CheckProgramCounter(uint32_t address)
{

    __eallow();

    //
    // Application is owner
    //
    HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_OWNER) = 1;

    //
    // Regular Masked Compare
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) =
            ((HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) & (~ERAD_HWBP_CNTL_COMP_MODE_M))|
                    ((uint16_t)0x0 << ERAD_HWBP_CNTL_COMP_MODE_S));

    //
    // Generate RTOS INT
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) |= ERAD_HWBP_CNTL_RTOSINT;

    //
    // Stop the CPU
    // Comment this line out to see the GPIO toggle in the
    // ISR while running the device
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) |= ERAD_HWBP_CNTL_STOP;

    //
    // VPC comparison for address
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) =
            ((HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) & (~ERAD_HWBP_CNTL_BUS_SEL_M))|
                    ((uint16_t)0x1 << ERAD_HWBP_CNTL_BUS_SEL_S));

    //
    // Clear HWBP Event
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CLEAR) |= ERAD_HWBP_CLEAR_EVENT_CLR;

    //
    // Do not set a mask
    //
    HWREG(ERAD_HWBP1_BASE + ERAD_O_HWBP_MASK) = 0x0;

    //
    // Set Reference Address
    //
    HWREG(ERAD_HWBP1_BASE + ERAD_O_HWBP_REF) = address;

    //
    // Enable HWBP
    //
    HWREGH(ERAD_GLOBAL_BASE + ERAD_O_GLBL_ENABLE) |= ERAD_GLBL_ENABLE_HWBP1;

    __edis();
}


//
// delay function
//
void delayFunction(void)
{
    uint32_t delay;
    for (delay = 1000; delay>1; delay--)
    {
        NOP;
    }
}

//
// RTOSISR - ISR for ROTS Interrupt
//
interrupt void
RTOSISR(void)
{
    GPIO_togglePin(0);

    __eallow();
    if (HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_STATUS) & ERAD_HWBP_STATUS_EVENT_FIRED)
    {
        HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CLEAR) |= ERAD_HWBP_CLEAR_EVENT_CLR;
    }
    __edis();
}

//
// End of File
//
