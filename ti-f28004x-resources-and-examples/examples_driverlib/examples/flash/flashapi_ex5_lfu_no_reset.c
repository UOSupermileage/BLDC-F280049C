//#############################################################################
//
// FILE:   flashapi_ex5_lfu_no_reset.c
//
// TITLE:  Live Firmware Update Example without device reset.
//
//! \addtogroup driver_example_list
//! <h1> Live Firmware Update Example </h1>
//!
//! This example demonstrates how to perform a live firmware update with use
//! of the Live Device Firmware Update (Live DFU or LDFU) command. The LDFU 
//! command is supported in Serial Flash Programmer which communicates with 
//! the SCI Flash Kernel.
//!
//! In the example, an SCI autobaud lock is performed and the byte used for autobaud
//! lock is echoed back. Two interrupts are initialized and enabled: SCI Rx FIFO
//! interrupt and CPU Timer 0 interrupt. The CPU Timer 0 interrupt occurs every 1
//! second; the interrupt service routine (ISR) for CPU Timer 0 toggles an LED
//! based on the build configuration that is running. LED1 is toggled for the
//! BANK0_FLASH and BANK0_ROM build configurations and LED2 is toggled for the
//! BANK1_FLASH and BANK1_ROM build configuration. The SCI Rx FIFO interrupt is set
//! for a FIFO interrupt level of 10 bytes. The number of bytes in a packet from the
//! Serial Flash Programmer (when using the LDFU command) is 10. When a command is
//! sent to the device from the Serial Flash Programmer, the SCI Rx FIFO ISR receives
//! a command from the 10 byte packet in the FIFO. If the command matches the Live
//! Device Firmware Update (Live DFU) command, then the code branches to the Live DFU
//! function located inside of the SCI Flash Kernel for the corresponding bank.
//!
//! The project contains 2 build configurations:
//!     - BANK0_FLASH: Links the program sections to the appropriate locations
//!                    in Bank 0 of flash and uses the Flash API library that
//!                    links the Flash API functions to flash. The 'codestart'
//!                    section is linked to the alternative flash entry point
//!                    for Bank 0 (0x8EFF0) and the rest of the sections are
//!                    linked to 0x082008 or above. Bank 0 configurations of
//!                    the flash kernel reserve sector 0, sector 1, and the
//!                    first 128 bits of sector 2 of Bank 0; therefore, sections
//!                    must be linked to 0x082008 or higher. After building the
//!                    configuration, the C2000 Hex Utility will output the
//!                    program in the appropriate SCI boot hex format for the
//!                    flash kernel and serial flash programmer in a file named
//!                    'flashapi_ex2_liveFirmwareUpdateBANK0FLASH.txt'.
//!
//!     - BANK1_FLASH: Links the program sections to the appropriate locations
//!                    in Bank 1 of flash and uses the Flash API library that
//!                    links the Flash API functions to flash. The 'codestart'
//!                    section is linked to the alternative flash entry point
//!                    for Bank 1 (0x9EFF0) and the rest of the sections are
//!                    linked to 0x092008 or above. Bank 1 configurations of
//!                    the flash kernel reserve sector 0, sector 1, and the
//!                    first 128 bits of sector 2 of Bank 1; therefore,
//!                    sections must be linked to 0x092008 or higher. After
//!                    building the configuration, the C2000 Hex Utility will
//!                    output the program in the appropriate SCI boot hex format
//!                    for the flash kernel and serial flash programmer in a
//!                    file named 'flashapi_ex2_liveFirmwareUpdateBANK1FLASH.txt'.
//!
//!	For more detailed steps and information refer to LFU_LED.docx placed at 
//! <C2000Ware>\device_support\f28004x\docs\LFU_LED_NO_RESET.pdf
//!
//! \b External \b Connections \n
//!  - Connect GPIO28 (SCI Rx) and GPIO29 (SCI Tx) to a COM port of the computer
//!    running the Serial Flash Programmer project. In control card this is 
//!    routed via the USB port. So juct connecting the USB cable to PC will 
//!    suffice.
//!
//! \b Watch \b LED1 or LED2 blinking and the blink rate. \n
//!  - None
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
#include "F021_F28004x_C28x.h"
#include "string.h"
//
// Defines
//

//
// Places a 1 for the reset bit of the SCIFFRX register
// 0x2000: 0010 0000 0000 0000
//           ^
//       (bit 13): RXFIFORESET
//
#define SCI_RXFF_RESET     0x2000U

//
// Macro defining LFU MODE, to be used to avoid reinit of variables and 
// peripherals after LFU switch. Set to a unique ID.
//
#define 	LFUMODE		0x5A5A

//
// Globals
//

//
// cpuTimer0IntCount: Counter used in the CPU Timer 0 ISR
//
uint16_t cpuTimer0IntCount;

//
// checksum: Accumulates a checksum of every byte that is received
//
uint16_t checksum;

//
// command: Stores the command received from a host's packet
//
uint16_t command;

//
// data: In the case that a command is sent with data, it is stored in the 'data' array
//
uint16_t data[10];

//
// length: In the case that a command is sent with data, 'length' stores
//         the length of the data sent from a host
//
uint16_t length;

//
// Variable to indicate that the LFU switch is in progress
//
uint16_t lfuSwitch;


//
// Function Prototypes
//
__interrupt void cpuTimer0ISR(void);
__interrupt void sciaRxISR(void);
void initCPUTimers(void);
void configCPUTimer(uint32_t, float, float);
void initFlashAPI(void);
void exampleError(Fapi_StatusType status);
uint16_t sciaGetOnlyWordData(void);
void sciaFlush(void);
void sendNAK(void);
void sendACK(void);
uint16_t sciGetPacket(uint16_t*, uint16_t*);
void configureToggleRate(void);

//
// Main
//
void main(void)
{
	if(lfuSwitch != LFUMODE)
	{
		//
		// Initialize device clock, PLL, and peripherals. Disable WD
		//
		Device_init();
		//
		// Initialize GPIO and configure the GPIO pin as a push-pull output
		//
		Device_initGPIO();
	}
    //
    // Configure LED1 for the BANK0 configuration
    //
    #ifdef BANK0
        GPIO_setPinConfig(DEVICE_GPIO_CFG_LED1);
        GPIO_setMasterCore(DEVICE_GPIO_PIN_LED1, GPIO_CORE_CPU1);
        GPIO_setPadConfig(DEVICE_GPIO_PIN_LED1, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED1, GPIO_DIR_MODE_OUT);

    //
    // Configure LED2 for the BANK1 configuration
    //
    #else

        GPIO_setPinConfig(DEVICE_GPIO_CFG_LED2);
        GPIO_setMasterCore(DEVICE_GPIO_PIN_LED2, GPIO_CORE_CPU1);
        GPIO_setPadConfig(DEVICE_GPIO_PIN_LED2, GPIO_PIN_TYPE_STD);
        GPIO_setDirectionMode(DEVICE_GPIO_PIN_LED2, GPIO_DIR_MODE_OUT);

    #endif
	
	if(lfuSwitch != LFUMODE)
	{	
		//
		// DEVICE_GPIO_PIN_SCITXDA is the SCI Tx pin.
		//
		GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1); 
		GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
		GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
		GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
		GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

		//
		// DEVICE_GPIO_PIN_SCIRXDA is the SCI Rx pin.
		//
		GPIO_setMasterCore(DEVICE_GPIO_PIN_SCIRXDA, GPIO_CORE_CPU1);
		GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
		GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
		GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
		GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);
	}
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initModule();
    Interrupt_initVectorTable();
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Register ISR for the CPU Timer interrupt
    //
    Interrupt_register(INT_TIMER0, &cpuTimer0ISR);

    //
    // Initialize CPU Timer 0
    //
	if(lfuSwitch != LFUMODE)
	{	
		initCPUTimers();
	}

	configureToggleRate();    

    //
    // Enable timer interrupt
    //
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);
	
	//
    // Map the ISR to the SCI wake interrupt.
    //
    Interrupt_register(INT_SCIA_RX, sciaRxISR);

    //
    // Enables CPU int1 which is connected to CPU-Timer 0
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    Interrupt_enable(INT_TIMER0);

    //
    // Starts CPU-Timer 0
    //
    CPUTimer_startTimer(CPUTIMER0_BASE);

    //
    // Enable interrupt in the PIE: Group 9 interrupt 1
    //
    Interrupt_enable(INT_SCIA_RX);

    //
    // Acknowledge interrupts in group 9
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);

    //
    // Enable Global Interrupt (INTM)
    //
    EINT;

    //
    // Perform software reset of the SCI port
    //
    SCI_performSoftwareReset(SCIA_BASE);

    //
    // Configure SCIA: 9600 baud, 1 stop bit, no parity
	// clock - 25000000
	// baud rate - 9600
    //
    SCI_setConfig(SCIA_BASE, 25000000, 9600, (SCI_CONFIG_WLEN_8 |
                                              SCI_CONFIG_STOP_ONE |
                                              SCI_CONFIG_PAR_NONE));
    //
    // Reset the SCI Tx and Rx channels
    //
    SCI_resetChannels(SCIA_BASE);

    //
    // Clear the SCI Rx FIFO interrupt status
    //
    SCI_clearInterruptStatus(SCIA_BASE, SCI_INT_RXFF);

    //
    // Enable SCI transmit and receive
    //
    SCI_enableModule(SCIA_BASE);

    //
    // Perform software reset of the SCI port
    //
    SCI_performSoftwareReset(SCIA_BASE);

    //
    // Hold the SCI Rx FIFO in reset by clearing the Rx FIFO reset bit
    //
    HWREG(SCIA_BASE + SCI_O_FFRX) &= ~SCI_RXFF_RESET;

    //
    // Disable the SCI Rx FIFO interrupt
    //
    SCI_disableInterrupt(SCIA_BASE, SCI_INT_RXFF);

    //
    // Enable the SCI Rx FIFO
    //
    SCI_enableFIFO(SCIA_BASE);

    //
    // Set the SCI FIFO interrupt level. Only the Rx FIFO interrupt is
    // registered with a FIFO interrupt level of 10 bytes
    //
    SCI_setFIFOInterruptLevel(SCIA_BASE, SCI_FIFO_TX16,
                                   SCI_FIFO_RX10);
    //
    // Enable the RXRDY interrupt.
    //
    SCI_enableInterrupt(SCIA_BASE, SCI_INT_RXFF);

    //
    // Perform an autobaud lock.
    // SCI expects an 'a' or 'A' to lock the baud rate.
    //
    SCI_lockAutobaud(SCIA_BASE);

    //
    // Store the character received in order to echo back
    //
    uint16_t byteData = SCI_readCharBlockingFIFO(SCIA_BASE);

    //
    // Echo back to the host
    //
    SCI_writeCharBlockingFIFO(SCIA_BASE, byteData);

    //
    // Initialize the flash API to ensure the Live DFU command can
    // write to flash
    //
	if(lfuSwitch != LFUMODE)
	{	
		initFlashAPI();
	}

	//
	// Clear lfuSwitch after use
	//
	if(lfuSwitch == LFUMODE)
    {
        lfuSwitch = 0;
    }

    //
    // Loop Forever
    //
    for(;;)
    {
	    //
		// If the command is the Live DFU command, branch to the appropriate
		// Live DFU function. The value of 0x0700 is the value of the Live DFU 
		// command.
		//
		// If the Live DFU command is received, then the appropriate Live DFU 
		// function is branched to in order to perform a firmware update. In 
		// the BANK0 build configuration, the program counter branches to the 
		// Live DFU function in Bank 0. In the BANK1 build configuration, the 
		// program counter branches to the Live DFU function in Bank 1.
		//

        // Since this command is implemented in the background loop, CPUTimer0ISR can still
        // run whenever the Timer interrupt occurs
		if(command == (uint16_t)0x0700)
		{
			command = 0;
			// LFU mode activated
			lfuSwitch = LFUMODE;
			SCI_disableInterrupt(SCIA_BASE, SCI_INT_RXFF);
			
			//
			// Disable the Rx FIFO; the flash kernel does not use the Rx FIFO
			// enhancement
			//
			SCI_disableFIFO(SCIA_BASE);

			//
			// If a BANK0 build configuration is chosen, branch to the Live DFU
			// command in Bank 0
			//
			#ifdef BANK0

				 //
				 // Branch to the address of the Live DFU function in Bank 0
				 //
				 asm("    LCR 0x81000");
				 
				 asm("    NOP");
				 asm("    NOP");
				 
				 //while(1);
				 // Turn off LED 1
		         CPUTimer_disableInterrupt(CPUTIMER0_BASE);
				 GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1);

				 asm("    NOP");
                 asm("    NOP");
                 asm("    NOP");
				 asm("    NOP");

				 // Jump to Bank 1 Image
				 asm("    LCR 0x9EFF0");

			#endif

			//
			// If a BANK1 build configuration is chosen, branch to the Live DFU
			// command in Bank 1
			//
			#ifdef BANK1

				 //
				 // Branch to the address of the Live DFU function in Bank 1
				 //
				 asm("    LCR 0x91000");
				 
				 asm("    NOP");
				 asm("    NOP");
				 
				 //while(1);
                 // Turn off LED 2
		         CPUTimer_disableInterrupt(CPUTIMER0_BASE);
                 GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1);
                 
                 asm("    NOP");
                 asm("    NOP");
                 asm("    NOP");
                 asm("    NOP");

				 // Jump to Bank 0 Image
				 asm("    LCR 0x8EFF0");

			#endif
			
		}

        //
		// Used so the background loop isn't empty
        //
        ;
    }
}

//
// initFlashAPI - Initializes the flash API
//
void initFlashAPI(void)
{
    //
    // Enable writing to the EALLOW protected registers
    //
    EALLOW;

    //
    // oReturnCheck is used to store the status of the flash API
    //
    Fapi_StatusType oReturnCheck;

    //
    // Initialize the flash API
    //
    oReturnCheck = Fapi_initializeAPI(F021_CPU0_BASE_ADDRESS, 100);

    //
    // Check the status of the flash API for an error
    //
    if(oReturnCheck != Fapi_Status_Success)
    {
        exampleError(oReturnCheck);
    }

    //
    // Initialize the Flash Memory Controller (FMC) and banks for an erase or
    // program command
    //
    oReturnCheck = Fapi_setActiveFlashBank(Fapi_FlashBank0);

    //
    // Check the status of the flash API for an error
    //
    if(oReturnCheck != Fapi_Status_Success)
    {
        exampleError(oReturnCheck);
    }

    //
    // Disable writing to the EALLOW protected registers
    //
    EDIS;
}

//
// initCPUTimers - Initializes CPU Timer 0.
//
void initCPUTimers(void)
{
    //
    // Initialize timer period to maximum
    //
    CPUTimer_setPeriod(CPUTIMER0_BASE, 0xFFFFFFFF);

    //
    // Initialize pre-scale counter to divide by 1 (SYSCLKOUT)
    //
    CPUTimer_setPreScaler(CPUTIMER0_BASE, 0);

    //
    // Make sure timer is stopped
    //
    CPUTimer_stopTimer(CPUTIMER0_BASE);

    //
    // Reload counter register with period value
    //
    CPUTimer_reloadTimerCounter(CPUTIMER0_BASE);

    //
    // Reset interrupt counter
    //
    cpuTimer0IntCount = 0;
}

//
// configCPUTimer - This function initializes the selected timer to the
//                  period specified by the "freq" and "period" parameters.
//                  The "freq" is entered as Hz and the period in uSeconds.
//                  The timer is held in the stopped state after
//                  configuration.
//
void configCPUTimer(uint32_t cpuTimer, float freq, float period)
{
    uint32_t temp;

    //
    // Initialize timer period:
    //
    temp = (uint32_t)(freq / 1000000 * period);
    CPUTimer_setPeriod(cpuTimer, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(cpuTimer, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(cpuTimer);
    CPUTimer_reloadTimerCounter(cpuTimer);
    CPUTimer_setEmulationMode(cpuTimer,
                              CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(cpuTimer);

    //
    // Resets interrupt counters for cpuTimer 0
    //
    cpuTimer0IntCount = 0;
}

//
// cpuTimer0ISR - The Interrupt Service Routine for CPU Timer 0: increments
//                cpuTimer0IntCount and turns on an LED if the value of
//                cpuTimer0IntCount is even; an LED is turned off if the value
//                is odd. This interrupt occurs every 1 second.
//
//                          Even value of cpuTimer0ISR     Odd value of cpuTimer0ISR
//                BANK0:         Turn on LED1                 Turn off LED1
//
//                BANK1:         Turn on LED2                 Turn off LED2
//
__interrupt void cpuTimer0ISR(void)
{
    //
    // Increment cpuTimer0IntCount
    //
    cpuTimer0IntCount++;

    //
    // If BANK0 build configuration is chosen, toggle LED1
    //
    #ifdef BANK0

        //
        // Turn on LED1 if cpuTimer0IntCount is even
        //
        if(cpuTimer0IntCount%2 == 0)
        {
            //
            // Turn on LED1
            //
            GPIO_writePin(DEVICE_GPIO_PIN_LED1, 0);
        }

        //
        // Turn off LED1 if cpuTimer0IntCount is not even
        //
        else
        {
            //
            // Turn off LED1
            //
            GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1);
        }

    //
    // If BANK1 build configuration is chosen, toggle LED2
    //
    #else

        //
        // Turn on LED2 if cpuTimer0IntCount is even
        //
        if(cpuTimer0IntCount%2 == 0)
        {
            //
            // Turn on LED2
            //
            GPIO_writePin(DEVICE_GPIO_PIN_LED2, 0);
        }

        //
        // Turn off LED2 if cpuTimer0IntCount is not even
        //
        else
        {
            //
            // Turn off LED2
            //
            GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1);
        }

    #endif

    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);
}

//
// sciaRxISR - Interrupt Service Routine for the FIFO receive interrupt:
//             receives a 10 byte packet from the host through the Rx FIFO.
//             A command is received from the packet. The command is stored 
//             in a variable and the same will be polled in main loop.
//
__interrupt void sciaRxISR(void)
{
    //
    // Receive a command from the host's 10 byte packet
    //
    command = sciGetPacket(&length, data);

    //
    // Hold the SCI Rx FIFO in reset by clearing the Rx FIFO reset bit
    //
    HWREG(SCIA_BASE + SCI_O_FFRX) &= ~SCI_RXFF_RESET;

    //
    // Enable the Rx FIFO. Takes the Rx FIFO out of a reset state.
    //
    SCI_enableFIFO(SCIA_BASE);

    //
    // Acknowledge the PIE interrupt.
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP9);
}


//
// exampleError - Error function that will halt the debugger when there is an
//                error while initializing the flash API
//
void exampleError(Fapi_StatusType status)
{
    //
    // Error code will be in the status parameter
    //
    __asm("    ESTOP0");
}

//
// sciaGetOnlyWordData - Fetches two bytes from the SCI-A
//                       port and puts them together to form a single
//                       16-bit value.  It is assumed that the host is
//                       sending the data in the order LSB followed by MSB.
//
uint16_t sciaGetOnlyWordData(void)
{
   uint16_t wordData;
   uint16_t byteData;

   wordData = 0x0000;
   byteData = 0x0000;

   //
   // Fetch the LSB and verify back to the host
   //
   wordData = SCI_readCharBlockingFIFO(SCIA_BASE);

   //
   // Fetch the MSB and verify back to the host
   //
   byteData = SCI_readCharBlockingFIFO(SCIA_BASE);

   //
   // Compute checksum.
   //
   checksum += wordData + byteData;

   //
   // Form the wordData from the MSB:LSB
   //
   wordData |= (byteData << 8);

   return wordData;
}

//
// sciGetPacket - Receives a packet from the host, returns the
//                command, and puts the data length in 'uint16_t* length'
//                and data in 'uint16_t* data'. The packet is received
//                from the SCI Rx FIFO.
//
uint16_t sciGetPacket(uint16_t* length, uint16_t* data)
{
    //
    // Check the packet header is correct
    //
    if(sciaGetOnlyWordData() != 0x1BE4)
    {
        //
        // Send NAK value to the host to indicate the packet header
        // was not correctly received
        //
        sendNAK();

        //
        // Start packet error
        //
        return(100);
    }

    //
    // Get the length of the data to be received
    //
    *length = sciaGetOnlyWordData();

    //
    // Initialize checksum before receiving the command and data; checksum
    // keeps a running total of each byte that is received
    //
    checksum = 0;

    //
    // Receive the command. Start accumulating a checksum of the command
    // and data
    //
    uint16_t command2 = sciaGetOnlyWordData();

    int i = 0;

    //
    // Receive data that the host may have sent.
    //
    for(i = 0; i < (*length)/2; i++)
    {
        *(data+i) = sciaGetOnlyWordData();
    }

    uint16_t dataChecksum = checksum;

    //
    // Compare application checksum to the checksum of the host
    //
    if(dataChecksum != sciaGetOnlyWordData())
    {
        //
        // Send NAK value to the host to indicate there is an
        // error with the checksum. The application checksum
        // doesn't match the host's checksum.
        //
        sendNAK();

        //
        // Checksum error
        //
        return(101);
    }
    if(sciaGetOnlyWordData() != 0xE41B)
    {
        //
        // Send NAK value to the host to indicate the end of the packet
        // was not correctly received
        //
        sendNAK();

        //
        // End packet error
        //
        return(102);
    }

    //
    // Acknowledge to the host that the packet was correctly received
    //
    sendACK();

    //
    // Return the command
    //
    return(command2);
}

//
// sendNAK - Transmits a NAK value to the host.
//
void sendNAK(void)
{
    //
    // Write Not Acknowledged (NAK) value to the host without using the FIFO
    //
    SCI_writeCharBlockingNonFIFO(SCIA_BASE, 0xA5);

    //
    // Wait for the Tx buffer to be empty
    //
    sciaFlush();
}

//
// sendACK - Transmits an ACK value to the host
//
void sendACK(void)
{
    //
    // Write Acknowledged (ACK) value to the host without using the FIFO
    //
    SCI_writeCharBlockingNonFIFO(SCIA_BASE, 0x2D);

    //
    // Wait for the Tx buffer to be empty
    //
    sciaFlush();
}

//
// sciaFlush - Waits until SCIA Tx is not busy
//
void sciaFlush(void)
{
    //
    // Wait while TX is busy.
    //
    while(SCI_isTransmitterBusy(SCIA_BASE))
    {
    }
}

void configureToggleRate(void)
{
	if(lfuSwitch != LFUMODE)
	{
		configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000000);		
	}
	
	if(lfuSwitch == LFUMODE)
	{
		configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 400000);		
	}
}

//
// End of File
//
