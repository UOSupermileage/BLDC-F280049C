//#############################################################################
//
// FILE:   flashapi_ex3_liveFirmwareUpdate.c
//
// TITLE:  Live Firmware Update Example
//
//! \addtogroup driver_example_list
//! <h1> Live Firmware Update Example </h1>
//!
//! This example demonstrates how to perform a live firmware update with use
//! of the Live Device Firmware Update (Live DFU or LDFU) command; it is to
//! be used with the Serial Flash Programmer as well as the SCI Flash Kernel.
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
//! The project contains 4 build configurations:
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
//!     - BANK0_ROM:   Links the program sections to the appropriate locations in
//!                    Bank 0 of flash. The sections are linked identical to that
//!                    of the BANK0_FLASH configuration.This build configuration
//!                    uses the Flash API library that uses the Flash API functions
//!                    from the ROM of the device. Revision A of F28004x does not
//!                    have the Flash API functions in ROM. After building the
//!                    configuration, the C2000 Hex Utility will output the
//!                    program in the appropriate SCI boot hex format for the
//!                    flash kernel and serial flash programmer in a file named
//!                    'flashapi_ex2_liveFirmwareUpdateBANK0ROM.txt'.
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
//!     - BANK1_ROM:   Links the program sections to the appropriate locations in
//!                    Bank 1 of flash. The sections are linked identical to that
//!                    of the BANK1_FLASH configuration.This build configuration
//!                    uses the Flash API library that uses the Flash API functions
//!                    from the ROM of the device. Revision A of F28004x does not
//!                    have the Flash API functions in ROM. After building the
//!                    configuration, the C2000 Hex Utility will output the
//!                    program in the appropriate SCI boot hex format for the
//!                    flash kernel and serial flash programmer in a file named
//!                    'flashapi_ex2_liveFirmwareUpdateBANK1ROM.txt'.
//!
//! To use the example, make sure line 867 is commented in the Serial Flash Programmer 
//! project in order to be able to load the kernel through CCS. Configure the device to boot to
//! flash at 0x80000 and follow the steps below:
//!     1. Place the output files of the C2000 Hex Utility tool for
//!        the build configurations to be used (one for each bank) in the 'hex' directory
//!        of the Serial Flash Programmer project.
//!     2. Load LDFU build configurations of the SCI Flash Kernel (one for each bank)
//!        to flash through CCS. If a debug session is launched in order to load a build
//!        configuration of the flash kernel to flash, go to Tools -> On-Chip Flash -> Erase Settings
//!        in order to ensure that 'Necessary Sectors Only (for Program Load)' is selected.
//!        This allows only the sectors where the flash kernel is linked to be erased when it is loaded.
//!        If 'Entire Flash' was selected, ensure that both build configurations are in flash.
//!     3. Run the bank 0 build configuration of the flash kernel in CCS, ensuring that the bank 1 build
//!        configuration is not erased.
//!     4. In the Serial Flash Programmer project, make sure the command arguments of
//!        the debug properties (Debug -> serial_flash_programmer Properties -> Debugging -> Command Arguments)
//!        contains the path to the hex formatted file of the bank 1 configuration of this
//!        example. Refer to the 'README.txt' file in the Serial Flash Programmer project to understand
//!        the command arguments of the debug properties. Verify that the correct COM port is selected.
//!     5. Run the Serial Flash Programmer project. A menu should appear inside a terminal.
//!     6. In the terminal, enter '8' to send the Live DFU command. The bank 1 build
//!        configuration of the example will start loading to the device and each byte that
//!        is sent will show in the terminal. Keep the terminal open until the file is done
//!        being loaded to the device, indicated by an "Application load successful!" message.
//!        Once the program is done loading, the bank 1 build configuration of the example
//!        will be running on the device.
//!     7. Exit the terminal and edit the command line arguments of the Serial Flash
//!        Programmer's debug properties so that it contains the path to the hex formatted
//!        file for the bank 0 build configuration of this example.
//!     8. Run the Serial Flash Programmer again in order to perform an autobaud lock
//!        and view the menu in the terminal. LED2 should be blinking to indicate
//!        that code is running on bank 1.
//!     8. Enter '8' in the terminal to send the Live DFU command to the device. The bank 0
//!        build configuration of the example will start loading to the device. Again, each
//!        byte that is sent will be shown in the terminal; the end of the loading is marked
//!        by an "Application load successful!" message. When the program is done loading,
//!        the bank 0 build configuration of this example should be running on the device.
//!     9. Exit the terminal and edit the command arguments of Serial Flash Programmer's
//!        debug properties so that it includes the path to the hex formatted file of the
//!        bank 1 configuration of this example.
//!    10. Run the Serial Flash Programmer to perform an autobaud lock and view the menu. LED1
//!        should be blinking to indicate that code is running on bank 0.
//!    11. Restart from step 6.
//!
//! If BANK1_FLASH and BANK0_FLASH build configurations are used for the example, then the
//! corresponding build configurations of the flash kernel must be loaded to flash: BANK1_LDFU
//! and BANK0_LDFU respectively. If BANK1_ROM and BANK0_ROM build configurations are used
//! for the example, then the corresponding build configurations of the flash kernel must
//! be loaded to flash: BANK1_LDFU_ROM and BANK0_LDFU_ROM respectively.
//!
//! \b External \b Connections \n
//!  - Connect GPIO28 (SCI Rx) and GPIO29 (SCI Tx) to a COM port of the computer
//!    running the Serial Flash Programmer project
//!
//! \b Watch \b Variables \n
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

//
// Main
//
void main(void)
{
    //
    // Initialize device clock, PLL, and peripherals. Disable WD
    //
    Device_init();

    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    Device_initGPIO();

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

    //
    // GPIO29 is the SCI Tx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_PIN_SCITXDA, GPIO_CORE_CPU1); 
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCITXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_DIR_MODE_OUT);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCITXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCITXDA, GPIO_QUAL_ASYNC);

    //
    // GPIO28 is the SCI Rx pin.
    //
    GPIO_setMasterCore(DEVICE_GPIO_CFG_SCIRXDA, GPIO_CORE_CPU1);
    GPIO_setPinConfig(DEVICE_GPIO_CFG_SCIRXDA);
    GPIO_setDirectionMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_DIR_MODE_IN);
    GPIO_setPadConfig(DEVICE_GPIO_PIN_SCIRXDA, GPIO_PIN_TYPE_STD);
    GPIO_setQualificationMode(DEVICE_GPIO_PIN_SCIRXDA, GPIO_QUAL_ASYNC);

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
    initCPUTimers();

    //
    // Configure CPU Timer 0 to interrupt every second:
    // 1000000 -> 1 second Period (in uSeconds)
    //
    configCPUTimer(CPUTIMER0_BASE, DEVICE_SYSCLK_FREQ, 1000000);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. If any of the configuration bits are changed
    // in configCPUTimer and initCPUTimers, the below settings must also
    // be updated.
    //
    CPUTimer_enableInterrupt(CPUTIMER0_BASE);

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
    // Map the ISR to the SCI wake interrupt.
    //
    Interrupt_register(INT_SCIA_RX, sciaRxISR);


    //
    // Initialize the flash API to ensure the Live DFU command can
    // write to flash
    //
    initFlashAPI();

    //
    // Loop Forever
    // This is the Background loop that executes when no ISRs are running
    //
    for(;;)
    {
        //
        // If the command is the Live DFU command, branch to the appropriate
        // Live DFU function. The value of 0x0700 is the value of the Live DFU command.
        //

        // Note that the below logic indicates that Ex3 only supports the LDFU command
        if(command == (uint16_t)0x0700)
        {
            // otherwise this logic will repeatedly execute
            command = 0;
            // no more SCI commands once LFU processing kicks off
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
                // LCR is used because we need to return here
                //
                 asm("    LCR 0x81000");
				 
                 // Turn off LED 1
                 CPUTimer_disableInterrupt(CPUTIMER0_BASE);
                 GPIO_writePin(DEVICE_GPIO_PIN_LED1, 1);

            #endif

            //
            // If a BANK1 build configuration is chosen, branch to the Live DFU
            // command in Bank 1
            //
            #ifdef BANK1

                 //
                 // Branch to the address of the Live DFU function in Bank 1
                 // LCR is used because we need to return here
                 //
                 asm("    LCR 0x91000");
				
                 // Turn off LED 2
                 CPUTimer_disableInterrupt(CPUTIMER0_BASE);
                 GPIO_writePin(DEVICE_GPIO_PIN_LED2, 1);

            #endif

            // After LFU command is completed, initiate a device reset through the Watchdog
            // Earlier this was happening in Ex2 in the Kernel, but with the addition of EX5 (LFU without device reset),
            // we are moving the WDG reset into this Example

            //
            // Set the watchdog to generate a reset signal instead of an
            // interrupt signal
            //
            SysCtl_setWatchdogMode(SYSCTL_WD_MODE_RESET);

            //
            // Enable the watchdog
            //
            SysCtl_enableWatchdog();
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
//             A command is received from the packet. If the Live DFU
//             command is received, then the appropriate Live DFU function
//             is branched to in order to perform a firmware update. In the BANK0
//             build configuration, the ISR branches to the Live DFU function
//             in Bank 0. In the BANK1 build configuration, the ISR branches
//             to the Live DFU function in Bank 1.
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

//
// End of File
//
