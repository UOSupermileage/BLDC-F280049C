//#############################################################################
//
// FILE:   flashapi_ex1_program_autoecc.c
//
// TITLE:  Flash programming example for AutoEcc option
//
//! \addtogroup driver_example_list
//! <h1> Flash Programming for AutoECC </h1>
//!
//! This example demonstrates how to program Flash using API's AutoEcc 
//! generation option.
//!
//! \b External \b Connections \n
//!  - None.
//!
//! \b Watch \b Variables \n
//!  - None.
//!
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
#include "driverlib.h"
#include "device.h"

//
// Include Flash API include file
//
#include "F021_F28004x_C28x.h"

//
// Include Flash API example header file
//
#include "flash_programming_f28004x.h"


//
// Defines
//

// Length (in 16-bit words) of data buffer used for program
#define  WORDS_IN_FLASH_BUFFER    0xFF

//
// Globals
//

//Data Buffers used for program operation using the flash API program function
#pragma  DATA_SECTION(Buffer,"DataBufferSection");
uint16   Buffer[WORDS_IN_FLASH_BUFFER + 1];
uint32   *Buffer32 = (uint32 *)Buffer;


//
// Prototype of the functions used in this example
//
void Example_Error(Fapi_StatusType status);
void Example_Done(void);
void Example_CallFlashAPI(void);
void FMSTAT_Fail(void);

//
// Main
//
void main(void)
{

	//
    // Initialize device clock and peripherals
	// Copy the Flash initialization code from Flash to RAM
    // Copy the Flash API from Flash to RAM
    // Configure Flash wait-states, fall back power mode, performance features and ECC
    //
    Device_init();

    //
    // Initialize GPIO
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    //  Notice that Flash API functions are mapped to RAM for execution in this example.
    //  In F28004x devices that have two banks, Flash API functions may be executed from
    //  one bank to perform Flash erase and program operations on the other bank.
    //  Flash API functions should not be executed from the same bank on which erase/
    //  program operations are in progress.
    //  Also, note that there should not be any access to the Flash bank on which erase/
    //  program operations are in progress.  Hence below function is mapped to RAM for
    //  execution.
    //
    Example_CallFlashAPI();

}


//*****************************************************************************
//  Example_CallFlashAPI
//
//  This function will interface to the flash API.
//  Flash API functions used in this function are executed from RAM in this
//  example.
//*****************************************************************************
#ifdef __cplusplus
#pragma CODE_SECTION(".TI.ramfunc");
#else
#pragma CODE_SECTION(Example_CallFlashAPI, ".TI.ramfunc");
#endif
void Example_CallFlashAPI(void)
{
    uint32 u32Index = 0;
    uint16 i = 0;
    Fapi_StatusType  oReturnCheck;
    Fapi_FlashStatusType  oFlashStatus;
    Fapi_FlashStatusWordType  oFlashStatusWord;

    //
    // Note that wait-states are already configured in the Device_init().
    // However, if INTOSC is used as the clock source and
    // if the CPUCLK falls in the range (97,100] (check other ranges given in DS),
    // then an extra wait state is needed for FSM operations (erase/program).
    // Hence, below function call should be uncommented in case INTOSC is used.
    // At 100MHz, execution wait-states for external oscillator is 4 and hence
    // in this example, a wait-state of 5 is used below.
    // This example is using external oscillator as the clock source and hence
    // below is commented.
    //
    // This wait-state setting impacts both Flash banks. Applications which
    // perform simultaneous READ/FETCH of one bank and PROGRAM or ERASE of the other
    // bank must use the higher RWAIT setting during the PROGRAM or ERASE operation. OR
    // use a clock source or frequency with a common wait state setting
    // Example: Use 97MHz instead of 100MHz if it is acceptable for the application.
    //
    // In case, if user application increments wait-state before using API,
    // then remember to revert back to the original wait-state after the API usage
    // to avoid extra wait-state during application execution from Flash.
    //
    //
    // Flash_setWaitstates(FLASH0CTRL_BASE, 5);

    // Initialize the Flash API by providing the Flash register base address
    // and operating frequency.
    // This function is required to initialize the Flash API based on System frequency
    // before any other Flash API operation can be performed.
    // This function must also be called whenever System frequency or RWAIT is changed.
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 100);

    if(oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }

    // Initialize the Flash banks and FMC for erase and program operations.
    // Fapi_setActiveFlashBank() function sets the Flash banks and FMC for further
    // Flash operations to be performed on the banks.
    // Note: It does not matter which bank is passed as the parameter to initialize.
    //       Both Banks and FMC get initialized with one function call unlike F2837xS.
    //       Hence there is no need to execute Fapi_setActiveFlashBank() for each bank.
    //       Executing for one bank is enough.
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    if(oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for possible errors
        Example_Error(oReturnCheck);
    }


    // Erase Flash Bank0 sector6
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                                        (uint32 *)Bzero_Sector6_start);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

	if(oReturnCheck != Fapi_Status_Success)
	{
		// Check Flash API documentation for possible errors
		Example_Error(oReturnCheck);
	}

    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    oFlashStatus = Fapi_getFsmStatus();
    if(oFlashStatus != 0)
    {
        // Check Flash API documentation for FMSTAT and debug accordingly
        // Fapi_getFsmStatus() function gives the FMSTAT register contents.
    	// Check to see if any of the EV bit, ESUSP bit, CSTAT bit or
    	// VOLTSTAT bit is set (Refer to API documentation for more details).
    	FMSTAT_Fail();
    }

    // Do blank check
    // Verify that Bank0 sector6 is erased.  The Erase command itself does a verify as
    // it goes.  Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_Sector6_start,
    		       Sector8KB_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for error info
        Example_Error(oReturnCheck);
    }


    // A data buffer of max 8 16-bit words can be supplied to the program function.
    // Each word is programmed until the whole buffer is programmed or a
    // problem is found. However to program a buffer that has more than 8
    // words, program function can be called in a loop to program 8 words for
    // each loop iteration until the whole buffer is programmed.
    //
    // Remember that the main array flash programming must be aligned to
    // 64-bit address boundaries and each 64 bit word may only be programmed
    // once per write/erase cycle.  Meaning the length of the data buffer
    // (3rd parameter for Fapi_issueProgrammingCommand() function) passed
    // to the program function can only be either 4 or 8.
    //
    // Program data in Flash using "AutoEccGeneration" option.
    // When AutoEccGeneration opton is used, Flash API calculates ECC for the given
    // 64-bit data and programs it along with the 64-bit main array data.
    // Note that any unprovided data with in a 64-bit data slice
    // will be assumed as 1s for calculating ECC and will be programmed.
    //
    // Note that data buffer (Buffer) is aligned on 64-bit boundary for verify reasons.
    //
    // Monitor ECC address for Bank0 Sector6 while programming with AutoEcc mode.
    //
    // In this example, 0xFF+1 bytes are programmed in Flash Bank0 Sector6
    // along with auto-generated ECC.

    //
    // Fill a buffer with data to program into the flash.
    //
    for(i=0; i <= WORDS_IN_FLASH_BUFFER; i++)
    {
        Buffer[i] = i;
    }

    for(i=0, u32Index = Bzero_Sector6_start;
       (u32Index < (Bzero_Sector6_start + WORDS_IN_FLASH_BUFFER)) &&
       (oReturnCheck == Fapi_Status_Success); i+= 8, u32Index+= 8)
    {
		oReturnCheck = Fapi_issueProgrammingCommand((uint32 *)u32Index, Buffer+i, 8,
																				 0, 0, Fapi_AutoEccGeneration);

		// Wait until the Flash program operation is over
		while(Fapi_checkFsmForReady() == Fapi_Status_FsmBusy);

		if(oReturnCheck != Fapi_Status_Success)
		{
			// Check Flash API documentation for possible errors
			Example_Error(oReturnCheck);
		}

		// Read FMSTAT register contents to know the status of FSM after
		// program command to see if there are any program operation related errors
		oFlashStatus = Fapi_getFsmStatus();
		if(oFlashStatus != 0)
		{
			//Check FMSTAT and debug accordingly
			FMSTAT_Fail();
		}

		// Verify the programmed values.  Check for any ECC errors.
		// The program command itself does a verify as it goes.
		// Hence program verify by CPU reads (Fapi_doVerify()) is optional.
        oReturnCheck = Fapi_doVerify((uint32 *)u32Index,
                                     4, Buffer32+(i/2),
                                     &oFlashStatusWord);

		if(oReturnCheck != Fapi_Status_Success)
		{
			// Check Flash API documentation for possible errors
			Example_Error(oReturnCheck);
		}
    }


	// Erase the sector that is programmed above
    // Erase Bank0 Sector6
    oReturnCheck = Fapi_issueAsyncCommandWithAddress(Fapi_EraseSector,
                   (uint32 *)Bzero_Sector6_start);

    // Wait until FSM is done with erase sector operation
    while (Fapi_checkFsmForReady() != Fapi_Status_FsmReady){}

	if(oReturnCheck != Fapi_Status_Success)
	{
		// Check Flash API documentation for possible errors
		Example_Error(oReturnCheck);
	}

    // Read FMSTAT register contents to know the status of FSM after
    // erase command to see if there are any erase operation related errors
    oFlashStatus = Fapi_getFsmStatus();
    if(oFlashStatus != 0)
    {
        // Check Flash API documentation for FMSTAT and debug accordingly
        // Fapi_getFsmStatus() function gives the FMSTAT register contents.
    	// Check to see if any of the EV bit, ESUSP bit, CSTAT bit or
    	// VOLTSTAT bit is set (Refer to API documentation for more details).
    	FMSTAT_Fail();
    }

    // Do blank check
    // Verify that Bank0 sector6 is erased.  The Erase command itself does a verify as
    // it goes.  Hence erase verify by CPU reads (Fapi_doBlankCheck()) is optional.
    oReturnCheck = Fapi_doBlankCheck((uint32 *)Bzero_Sector6_start,
    		       Sector8KB_u32length,
                   &oFlashStatusWord);

    if(oReturnCheck != Fapi_Status_Success)
    {
        // Check Flash API documentation for error info
        Example_Error(oReturnCheck);
    }

    // In case, if user application increments wait-state before using API
    // for INTOSC reason, then remember to revert back (uncomment below funcion call)
    // to the original wait-state after the API usage to avoid extra wait-state
    // during application execution from Flash.
    // At 100MHz, execution wait-states is 4 and hence in this example,
    // a wait-state of 4 is used below.
    //
    // Flash_setWaitstates(FLASH0CTRL_BASE, 4);

    // Example is done here
    Example_Done();
}

//******************************************************************************
// For this example, just stop here if an API error is found
//******************************************************************************
void Example_Error(Fapi_StatusType status)
{
    //  Error code will be in the status parameter
        __asm("    ESTOP0");
}

//******************************************************************************
//  For this example, once we are done just stop here
//******************************************************************************
void Example_Done(void)
{
    __asm("    ESTOP0");
}

//******************************************************************************
// For this example, just stop here if FMSTAT fail occurs
//******************************************************************************
void FMSTAT_Fail(void)
{
    //  Error code will be in the status parameter
        __asm("    ESTOP0");
}


//
// End of File
//
