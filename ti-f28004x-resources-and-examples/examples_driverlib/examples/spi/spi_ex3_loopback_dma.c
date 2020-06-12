//#############################################################################
//
// FILE:   spi_ex3_loopback_dma.c
//
// TITLE:  SPI Digital Loopback with DMA
//
//! \addtogroup driver_example_list
//! <h1>SPI Digital Loopback with DMA</h1>
//!
//! This program uses the internal loopback test mode of the SPI module. Both
//! DMA interrupts and the SPI FIFOs are used. When the SPI transmit FIFO has
//! enough space (as indicated by its FIFO level interrupt signal), the DMA
//! will transfer data from global variable sData into the FIFO. This will be
//! transmitted to the receive FIFO via the internal loopback.
//!
//! When enough data has been placed in the receive FIFO (as indicated by its
//! FIFO level interrupt signal), the DMA will transfer the data from the FIFO
//! into global variable rData.
//!
//! When all data has been placed into rData, a check of the validity of the
//! data will be performed in one of the DMA channels' ISRs.
//!
//! \b External \b Connections \n
//!  - None
//!
//! \b Watch \b Variables \n
//!  - \b sData - Data to send
//!  - \b rData - Received data
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
// Defines
//
#define FIFO_LVL    8               // FIFO interrupt level
#define BURST       FIFO_LVL        // Each burst will empty the FIFO
#define TRANSFER    16              // It will take 16 bursts of 8 to transfer
                                    // all data in rData

//
// Globals
//
uint16_t sData[128];                // Send data buffer
uint16_t rData[128];                // Receive data buffer

// Place buffers in GSRAM
#pragma DATA_SECTION(sData, "ramgs0");
#pragma DATA_SECTION(rData, "ramgs1");

volatile uint16_t done = 0;         // Flag to set when all data transfered

//
// Function Prototypes
//
void initDMA(void);
void initSPIFIFO(void);
__interrupt void dmaCh5ISR(void);
__interrupt void dmaCh6ISR(void);

//
// Main
//
void main(void)
{
    uint16_t i;

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
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
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_DMA_CH5, &dmaCh5ISR);
    Interrupt_register(INT_DMA_CH6, &dmaCh6ISR);

    //
    // Set up DMA for SPI use, initialize the SPI for FIFO mode
    //
    initDMA();
    initSPIFIFO();

    //
    // Initialize the data buffers
    //
    for(i = 0; i < 128; i++)
    {
        sData[i] = i;
        rData[i]= 0;
    }

    //
    // Enable interrupts required for this example
    //
    Interrupt_enable(INT_DMA_CH5);
    Interrupt_enable(INT_DMA_CH6);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Start the DMA channels
    //
    DMA_startChannel(DMA_CH6_BASE);
    DMA_startChannel(DMA_CH5_BASE);

    //
    // Wait until the DMA transfer is complete
    //
    while(!done);

    //
    // When the DMA transfer is complete the program will stop here
    //
    ESTOP0;
}

//
// Function to configure SPI A in FIFO mode.
//
void initSPIFIFO()
{
    //
    // Must put SPI into reset before configuring it
    //
    SPI_disableModule(SPIA_BASE);

    //
    // FIFO configuration
    //
    SPI_enableFIFO(SPIA_BASE);
    SPI_clearInterruptStatus(SPIA_BASE, SPI_INT_RXFF | SPI_INT_TXFF);
    SPI_setFIFOInterruptLevel(SPIA_BASE, (SPI_TxFIFOLevel)FIFO_LVL,
                             (SPI_RxFIFOLevel)FIFO_LVL);


    //
    // SPI configuration. Use a 500kHz SPICLK and 16-bit word size.
    //
    SPI_setConfig(SPIA_BASE, DEVICE_LSPCLK_FREQ, SPI_PROT_POL0PHA0,
                  SPI_MODE_MASTER, 500000, 16);
    SPI_enableLoopback(SPIA_BASE);
    SPI_enableModule(SPIA_BASE);
}

//
// DMA setup for both TX and RX channels.
//
void initDMA()
{
    //
    // Initialize DMA
    //
    DMA_initController();

    //
    // Configure DMA Ch5 for TX. When there is enough space in the FIFO, data
    // will be transferred from the sData buffer to the SPI module's transmit
    // buffer register.
    //
    DMA_configAddresses(DMA_CH5_BASE, (uint16_t *)(SPIA_BASE + SPI_O_TXBUF),
                        sData);
    DMA_configBurst(DMA_CH5_BASE, BURST, 1, 0);
    DMA_configTransfer(DMA_CH5_BASE, TRANSFER, 1, 0);
    DMA_configMode(DMA_CH5_BASE, DMA_TRIGGER_SPIATX, DMA_CFG_ONESHOT_DISABLE |
                   DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT);

    //
    // Configure DMA Ch5 interrupts
    //
    DMA_setInterruptMode(DMA_CH5_BASE, DMA_INT_AT_END);
    DMA_enableInterrupt(DMA_CH5_BASE);
    DMA_enableTrigger(DMA_CH5_BASE);

    //
    // Configure DMA Ch6 for RX. When the FIFO contains at least 8 words to
    // read, data will be transferred from the SPI module's receive buffer
    // register to the rData buffer.
    //
    DMA_configAddresses(DMA_CH6_BASE, rData,
                        (uint16_t *)(SPIA_BASE + SPI_O_RXBUF));
    DMA_configBurst(DMA_CH6_BASE, BURST, 0, 1);
    DMA_configTransfer(DMA_CH6_BASE, TRANSFER, 0, 1);
    DMA_configMode(DMA_CH6_BASE, DMA_TRIGGER_SPIARX, DMA_CFG_ONESHOT_DISABLE |
                   DMA_CFG_CONTINUOUS_DISABLE | DMA_CFG_SIZE_16BIT);

    //
    // Configure DMA Ch6 interrupts
    //
    DMA_setInterruptMode(DMA_CH6_BASE, DMA_INT_AT_END);
    DMA_enableInterrupt(DMA_CH6_BASE);
    DMA_enableTrigger(DMA_CH6_BASE);
}

//
// DMA Channel 5 ISR
//
__interrupt void dmaCh5ISR(void)
{
    DMA_stopChannel(DMA_CH5_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
    return;
}

//
// DMA Channel 6 ISR
//
 __interrupt void dmaCh6ISR(void)
{
    uint16_t i;

    DMA_stopChannel(DMA_CH6_BASE);
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);

    //
    // Check for data integrity
    //
    for(i = 0; i < 128; i++)
    {
        if (rData[i] != i)
        {
            // Something went wrong. rData doesn't contain expected data.
            ESTOP0;
        }
    }

    done = 1;
    return;
}
