//###########################################################################
//
// FILE:   flashapi_ex2_sciKernel.c
//
// TITLE: SCI Flash Kernel Example for F28004x
//
//! \addtogroup cpu01_example_list
//! <h1>SCI Flash Kernel Example for F28004x</h1>
//!
//! In this example, we set up a UART connection with a host using SCI, receive
//! commands for CPU1 to perform which then sends ACK, NAK, and status packets
//! back to the host after receiving and completing the tasks. Each command
//! either expects no data from the command packet or specific data relative
//! to the command.
//!
//! The example has seven build configurations, listed below.
//!         BANK0_LDFU: Only supports the Live Device Firmware Update (LDFU)
//!                     command for Bank 0; other commands are not supported
//!                     by this build configuration. For more information about
//!                     the LDFU command functionality, please read the documentation 
//!                     in 'flashapi_ex2_LDFU.c'. Links the flash kernel to Bank 0
//!                     (0x80000 - 0x82007 will be reserved). Uses Flash API symbols 
//!                     in flash.
//!                     
//!         BANK0_LDFU_ROM: Only supports the Live Device Firmware Update (LDFU)
//!                         command for Bank 0; other commands are not supported
//!                         by this build configuration. For more information about
//!                         the LDFU command functionality, please read the documentation 
//!                         in 'flashapi_ex2_LDFU.c'. Links the flash kernel to Bank 0
//!                         (0x80000 - 0x82007 will be reserved). Uses Flash API symbols 
//!                         in ROM; Rev A of F28004x cannot be used with this build  
//!                         configuration.
//!                         
//!         BANK0_NO_LDFU: Supports all commands except the LDFU command. Links the
//!                        flash kernel to Bank 0. Uses Flash API symbols in flash.
//!                        Commands must only be used to manipulate flash in Bank 1.
//!         
//!         BANK1_LDFU: Only supports the Live Device Firmware Update (LDFU)
//!                     command for Bank 1; other commands are not supported
//!                     by this build configuration. For more information about
//!                     the LDFU command functionality, please read the documentation 
//!                     in 'flashapi_ex2_LDFU.c'. Links the flash kernel to Bank 1
//!                     (0x90000 - 0x92007 will be reserved).Uses Flash API symbols in   
//!                     flash.
//!
//!         BANK1_LDFU_ROM: Only supports the Live Device Firmware Update (LDFU)
//!                         command for Bank 1; other commands are not supported
//!                         by this build configuration. For more information about
//!                         the command functionality, please read the documentation 
//!                         in 'flashapi_ex2_LDFU.c'. Links the flash kernel to Bank 1.
//!                         (0x90000 - 0x92007 will be reserved) Uses Flash API  
//!                         symbols in ROM; Rev A of F28004x cannot be used with this 
//!                         build configuration.
//!                         
//!         BANK1_NO_LDFU: Supports all commands except the LDFU command. Links the
//!                        flash kernel to Bank 1.  Uses Flash API symbols in flash.
//!                        Commands must only be used to manipulate flash in Bank 0.
//!
//!         CPU1_RAM: Supports all commands except the LDFU command. Links the flash
//!                   kernel to RAM.
//!
//! This example is to be used with the Serial Flash Programmer. Applications loaded to
//! flash through commands of the Serial Flash Programmer must be in the '--boot --sci8
//! --ascii' hex format, which can be achieved by using the hex utility tool.
//!
//! \b External \b Connections \n
//!  - Connect SCI Rx and SCI Tx to a COM port of the computer
//!    running the Serial Flash Programmer project
//!
//! \b Watch \b Variables \n
//!  - None
//!          
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

//
// Includes
//
#include <string.h>
#include "flashapi_ex2_erase.h"
#include "flashapi_ex2_flash_programming_c28.h"
#include "F021_F28004x_C28x.h"
#include "flashapi_ex2_bootrom.h"
#include "device.h"
#include "flash.h"
#include "gpio.h"
#include "driverlib.h"

//
// Function Prototypes
//
void exampleError(Fapi_StatusType status);
void initFlashSectors(void);
extern uint32_t sciGetFunction(uint32_t  BootMode);
extern void sciaFlush(void);

//
// main - This is an example code demonstrating F021 Flash API usage.
//        This code is in Flash
//

uint32_t main(void)
{
    //
    // flush SCIA TX port by waiting while it is busy, driverlib.
    //
    sciaFlush();

    //
    // initialize device and GPIO, driverlib.
    //
    Device_init();
    Device_initGPIO();

    //
    // init interrupt and vectorTable, drivelib.
    //
    Interrupt_initModule();
    Interrupt_initVectorTable();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // initialize flash_sectors, fapi + driverlib
    //
    initFlashSectors();

    uint32_t EntryAddr;

    //
    // parameter SCI_BOOT for GPIO28 (RX),29 (TX) is default.
    //
    EntryAddr = sciGetFunction(SCI_BOOT);
    return(EntryAddr);
}


//
// initFlashSectors - Initializes the flash API
//
void initFlashSectors(void)
{
    //
    // For the Live DFU configurations, error correction is not disabled when
    // initializing the flash API
    //
    #ifndef LIVE_UPDATE

        //
        // disable Error correction, driverlib.
        //
        Flash_disableECC(FLASH0ECC_BASE);

    #endif
    EALLOW;

    Fapi_StatusType oReturnCheck;
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 100);
    if(oReturnCheck != Fapi_Status_Success)
    {
        exampleError(oReturnCheck);
    }
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);
    if(oReturnCheck != Fapi_Status_Success)
    {
        exampleError(oReturnCheck);
    }

    EDIS;
    
    //
    // For the Live DFU configurations, error correction is not disabled when
    // initializing the flash API, so there is no need to re-enable it
    //
    #ifndef LIVE_UPDATE

        //
        // enable Error correction, driverlib
        //
        Flash_enableECC(FLASH0ECC_BASE);

    #endif
}

//
// Functions are not placed in RAM for the Live DFU configurations
//
#ifndef LIVE_UPDATE

#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(exampleError,".TI.ramfunc");
#endif

#endif

//
// Example_Error - Error function that will halt debugger
//
void exampleError(Fapi_StatusType status)
{
    //
    // Error code will be in the status parameter
    //
    __asm("    ESTOP0");
}

//
// End of file
//
