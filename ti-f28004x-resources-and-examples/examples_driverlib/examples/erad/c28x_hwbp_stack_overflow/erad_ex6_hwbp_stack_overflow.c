//#############################################################################
//
// FILE:   erad_ex6_hwbp_stack_overflow.c
//
// TITLE:  ERAD HWBP Stack Overflow Detection.
//
//! \addtogroup driver_example_list
//! <h1>ERAD HWBP Stack Overflow Detection</h1>
//!
//!  This example uses HWBP1 to monitor the STACK. The HWBP is set to monitor
//!  the data write access bus and STOP (HALT) the CPU when an access is
//!  detected to end of the STACK.
//!
//!  \b Watch \b Variables \n
//!  - functionCallCount - the number of times the recursive function
//!    overflowing the STACK is called.
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
// Check the MAP file for "__STACK_END" to find the address
// for the end of stack
//
#define STACK_END_ADDRESS   0x00000800UL

//
// Global Variables
//
volatile uint32_t functionCallCount = 0;


//
// Function Prototypes
//
void recursiveFunction(uint32_t delay);
void initHWBP1CheckDataWriteBus(uint32_t address);

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
    // Setup HWBP1 to monitor the end of the STACK
    //
    initHWBP1CheckDataWriteBus((uint32_t)STACK_END_ADDRESS - 1);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    recursiveFunction(1);

    //
    // IDLE loop. Just sit and loop forever (optional)
    //
    while(1)
    {

    }
}

//
// HWBP to halt the device
//
void initHWBP1CheckDataWriteBus(uint32_t address)
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
    // Stop the CPU
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) |= ERAD_HWBP_CNTL_STOP;

    //
    // DWAB comparison for address
    //
    HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) =
            ((HWREGH(ERAD_HWBP1_BASE + ERAD_O_HWBP_CNTL) & (~ERAD_HWBP_CNTL_BUS_SEL_M))|
                    ((uint16_t)0x2 << ERAD_HWBP_CNTL_BUS_SEL_S));

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
void recursiveFunction(uint32_t delay)
{
    //
    // If the debugger halted here, then the Stack overflow has occurred
    //
    functionCallCount++;

    //
    // Recursive function
    //
    recursiveFunction(delay + 1UL);
}

//
// End of File
//
