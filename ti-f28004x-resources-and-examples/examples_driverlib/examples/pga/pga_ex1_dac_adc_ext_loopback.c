//#############################################################################
//
// FILE:   pga_ex1_dac_adc_ext_loopback.c
//
// TITLE:   PGA DAC-ADC External Loopback Example.
//
//! \addtogroup driver_example_list
//! <h1>PGA DAC-ADC External Loopback Example</h1>
//!
//! This example generates 400 mV using the DAC output (it uses an internal
//! voltage reference). The output of the DAC is externally connected to PGA2
//! for a 3x gain amplification. It uses two ADC channels to sample the
//! DAC output and the amplified voltage output from PGA2. The ADC is connected
//! to these signals internally.
//!
//! \b External \b Connections on Control Card\n
//!  - Connect DACA_OUT (Analog Pin A0) to PGA2_IN (Pin 21 on HSEC connector of
//!  - ControlCard)
//!  - Connect PGA246NEG to GND
//!
//! \b External \b Connections on Launchpad\n
//!  - Connect DACA_OUT (Analog Pin A0) to PGA2_IN (Analog Pin A3).
//!  - Connect PGA246NEG to GND
//!
//! \b Watch \b Variables \n
//!  - \b dacResult - The DAC output voltage.
//!  - \b pgaResult - The amplified DAC voltage.
//!  - \b pgaGain   - The ratio of the amplified DAC voltage to the original
//!                   DAC output. This should always read a value of ~3.0.
//!
//
//#############################################################################
// $TI Release:
// $Release Date:
// $Copyright:
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"

//
// Globals
//
volatile uint16_t dacResult;
volatile uint16_t pgaResult;
float pgaGain;

//
// Function Prototypes
//
void initPGA(void);
void initDAC(void);
void initADC(void);
void initADCSOC(void);

//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull ups.
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
    // Initialize PGA, DAC, and ADC
    //
    initPGA();

    initDAC();
    DAC_setShadowValue(DACA_BASE, 600U);

    initADC();
    initADCSOC();

    DEVICE_DELAY_US(3000U);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    //
    // Force ADC conversion
    //
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);
    ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);

    //
    // Loop indefinitely
    //
    while(1)
    {
        //
        // Check if conversion is complete
        //
        if(ADC_getInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1))
        {
            //
            // Acknowledge flag
            //
            ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

            dacResult = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
            pgaResult = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER1);
            pgaGain   = (float)pgaResult / (float)dacResult;

            //
            // Force ADC conversion
            //
            ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER0);
            ADC_forceSOC(ADCA_BASE, ADC_SOC_NUMBER1);
        }
    }
}

//
// initADC - Configure ADC
//
void initADC(void)
{

    //
    // Set the voltage reference
    //
    // NOTE:
    // VREFHI pin must not be driven by an external reference
    // voltage. Disconnect any external reference before proceeding
    //
    ESTOP0;
    ADC_setVREF(ADCA_BASE, ADC_REFERENCE_INTERNAL, ADC_REFERENCE_3_3V);

    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    //
    // Flag is raised at the end of conversion
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Enable ADC
    //
    ADC_enableConverter(ADCA_BASE);

    DEVICE_DELAY_US(1000U);
}

//
// initADCSOC - Configure ADC SOCs
//
void initADCSOC(void)
{
    //
    // Convert DACA output
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0,
                 ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN0, 10U);

    //
    // Convert PGA2 output
    //
    ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER1,
                 ADC_TRIGGER_SW_ONLY, ADC_CH_ADCIN11, 10U);

    //
    // Configure ADC interrupt status flags
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

//
// initDAC - Configure DAC
//
void initDAC(void)
{
    //
    // Set the DAC gain to 2
    //
    DAC_setGainMode(DACA_BASE, DAC_GAIN_TWO);

    //
    // Use ADC voltage reference
    //
    DAC_setReferenceVoltage(DACA_BASE, DAC_REF_ADC_VREFHI);

    //
    // Load count value for DAC on next SYSCLK
    //
    DAC_setLoadMode(DACA_BASE, DAC_LOAD_SYSCLK);

    //
    // Enable DAC output
    //
    DAC_enableOutput(DACA_BASE);
}

//
// initPGA - Configure PGA2 gain
//
void initPGA(void)
{
    //
    // Set a gain of 3 to PGA2
    //
    PGA_setGain(PGA2_BASE, PGA_GAIN_3);

    //
    // No filter resistor for output
    //
    PGA_setFilterResistor(PGA2_BASE, PGA_LOW_PASS_FILTER_DISABLED);

    //
    // Enable PGA2
    //
    PGA_enable(PGA2_BASE);
}
