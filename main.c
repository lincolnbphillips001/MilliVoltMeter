//*****************************************************************************
//
// main.c - MilliVolt meter.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <float.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/fpu.h"
#include "driverlib/interrupt.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

/* Definitions */
#define LED_RED     GPIO_PIN_1
#define LED_BLUE    GPIO_PIN_2
#define LED_GREEN   GPIO_PIN_3

#define SERIAL_DATA_LOW             0
#define SERIAL_DATA_HIGH            1
#define SERIAL_DATA_BUFFER_LENGTH   32
#define MAX_ADC_VALUE               0xFFFFFF
#define VOLTAGE_REF                 4.096
#define MAX_VOLTAGE_VALUE           50
#define MIN_VOLTAGE_VALUE           0
#define OUTPUT_BUFFER_LENGTH        32

#define ADC_CS_LOW                  0
#define ADC_CS_HIGH                 1
#define DEBUG_LOW                   0
#define DEBUG_HIGH                  1
#define USER_INPUT_INT_TRIGGER_COUNTER_LIMIT    400

/* Global variables */
volatile uint32_t g_ui32SerialDataState;
volatile uint32_t g_ui32SerialDataCounter;
volatile uint32_t g_ui32SerialData[SERIAL_DATA_BUFFER_LENGTH];
volatile uint8_t g_ui8DebugOutputState;
volatile uint32_t g_ui32UserInputIntTrigger;
volatile uint32_t g_ui32CalibrationButtonPressed;

/* Forward Prototypes */
void adc_clock_interrupt_handler(void);
void user_input_interrupt_handler(void);
void config_porta_interrupts(void);
void config_porte_interrupts(void);
void config_system_clock(void);
void config_fpu(void);
void config_interrupts(void);
void configure_adc_chip_select(void);
void configure_debug(void);
void configure_uart(void);
uint32_t get_adcvalue(void);
float get_voltage(uint32_t);
void set_adc_chip_select(uint8_t);
void set_debug_output(uint8_t);
void release_user_input_interrupt_triggers(void);


//*****************************************************************************
//
// This is the handler for INT_GPIOA.  It simply saves the interrupt sequence
// number.
//
//*****************************************************************************
void
adc_clock_interrupt_handler(void)
{
    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2);

    //what is the state on the serial data pin (High or Low)
    g_ui32SerialDataState = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_3);

    //shift data to create a 1 or 0
    g_ui32SerialDataState = g_ui32SerialDataState >> 3;

    switch(g_ui32SerialDataState) {

    case SERIAL_DATA_LOW:
        g_ui32SerialData[g_ui32SerialDataCounter] = SERIAL_DATA_LOW;
        break;

    case SERIAL_DATA_HIGH:
        g_ui32SerialData[g_ui32SerialDataCounter] = SERIAL_DATA_HIGH;
        break;
    }

    g_ui32SerialDataCounter++;

}

void
user_input_interrupt_handler(void)
{
    GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_0);

    if (g_ui32UserInputIntTrigger == true) {
        g_ui32CalibrationButtonPressed = true;
        g_ui32UserInputIntTrigger = false;
    }
}

void
config_system_clock(void)
{
    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ |
                   SYSCTL_OSC_MAIN);
}

void
config_fpu(void)
{
    FPUEnable();
}

void
config_porta_interrupts(void)
{

    //
    // Enable the GPIOA peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Wait for the GPIOA module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }

    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    GPIOIntRegister(GPIO_PORTA_BASE, adc_clock_interrupt_handler);

    //
    // Initialize the GPIO pin configuration.
    // Set pins 2 and 3 as input, SW controlled.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    //
    // Sets the pad configuration for the specified pin(s).
    //
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_3, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    //
    // Make pin 2 falling edge triggered interrupts.
    //
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);

    //
    // Enable the pin interrupts.
    //
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Register the interrupt handler function for INT_GPIOA.
    //
    IntRegister(INT_GPIOA, adc_clock_interrupt_handler);
    //
    // Enable the interrupt for INT_GPIOA.
    //
    IntEnable(INT_GPIOA);
}

void
config_porte_interrupts(void)
{

    //
    // Enable the GPIOE peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    //
    // Wait for the GPIOE module to be ready.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }

    //
    // Register the port-level interrupt handler. This handler is the first
    // level interrupt handler for all the pin interrupts.
    //
    GPIOIntRegister(GPIO_PORTE_BASE, user_input_interrupt_handler);

    //
    // Initialize the GPIO pin configuration.
    // Set pin 0 as input, SW controlled.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_0);

    //
    // Sets the pad configuration for the specified pin(s).
    //
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    //
    // Make pin 0 falling edge triggered interrupts.
    //
    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_0, GPIO_RISING_EDGE);

    //
    // Enable the pin interrupts.
    //
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_0);

    //
    // Register the interrupt handler function for INT_GPIOE.
    //
    IntRegister(INT_GPIOE, user_input_interrupt_handler);
    //
    // Enable the interrupt INT_GPIOE.
    //
    IntEnable(INT_GPIOE);
}

void
config_interrupts(void)
{
    config_porta_interrupts();
    config_porte_interrupts();

    //
    // Enable master interruptsUART 0.
    //
    IntMasterEnable();
}

void
configure_adc_chip_select(void)
{
    // At this point is the code PortA will have been initialized in the function
    // config_interrupts();

    // Set pin 4 as output, SW controlled.
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);

    set_adc_chip_select(ADC_CS_HIGH);

}

void
configure_debug(void)
{
    // At this point is the code PortA will have been initialized in the function
    // config_interrupts();

    // Set pin 5 as output, SW controlled.
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);

    // Make pin 5 low
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);

    g_ui8DebugOutputState = DEBUG_LOW;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
configure_uart(void)
{

    //
    // Enable UART0
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void
ms_delay(uint32_t delayInMilliSeconds)
{
    static uint32_t divider;
    float fTime;
    float fFrequency;
    static uint8_t oneShot = true;

    // This is based on:-
    // SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_XTAL_16MHZ |
    // SYSCTL_OSC_MAIN);

    //(ms / 1000) = T
    //1 / T = f

    if (oneShot == true) {
        fTime = (float)delayInMilliSeconds / 1000;
        fFrequency = 1 / fTime;
        //convert float to uint32_t
        divider = (uint32_t)fFrequency;
        oneShot = false;
    }

    //
    // Delay for x milliSeconds
    //
    SysCtlDelay((SysCtlClockGet()/3) / divider);


}

uint32_t
get_adcvalue(void)
{
    uint8_t ui8SerialData[SERIAL_DATA_BUFFER_LENGTH];
    uint8_t indexA;
    uint32_t ui32RawDataValue;

    // copy global data to local data
    for (indexA = 0; indexA < SERIAL_DATA_BUFFER_LENGTH; indexA++) {
        ui8SerialData[indexA] = (uint8_t)g_ui32SerialData[indexA];
    }
    // clear global data
    for (indexA = 0; indexA < SERIAL_DATA_BUFFER_LENGTH; indexA++) {
        g_ui32SerialData[indexA] = 0;
    }
    //reset g_ui32SerialDataCounter counter
    g_ui32SerialDataCounter = 0;

    ui32RawDataValue = 0;

    //ui8SerialData[0] = 0; //EOC
    //ui8SerialData[1] = 0; //DMY
    //ui8SerialData[2] = 1; //SIG

    ui32RawDataValue = ui8SerialData[3] << 24; //EXR
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[4] << 23); //MSB - Bit27
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[5] << 22); //Bit26
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[6] << 21); //Bit25
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[7] << 20); //Bit24

    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[8] << 19); //Bit23
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[9] << 18); //Bit22
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[10] << 17); //Bit21
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[11] << 16); //Bit20
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[12] << 15); //Bit19
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[13] << 14); //Bit18
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[14] << 13); //Bit17
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[15] << 12); //Bit16

    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[16] << 11); //Bit15
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[17] << 10); //Bit14
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[18] << 9); //Bit13
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[19] << 8); //Bit12
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[20] << 7); //Bit11
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[21] << 6); //Bit10
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[22] << 5); //Bit09
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[23] << 4); //Bit08

    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[24] << 3); //Bit07
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[25] << 2); //Bit06
    ui32RawDataValue = ui32RawDataValue | (ui8SerialData[26] << 1); //Bit05
    ui32RawDataValue = ui32RawDataValue | ui8SerialData[27]; //LSB - Bit04
    //ui8SerialData[28] = 1; //SUB LSB 3 - Bit03
    //ui8SerialData[29] = 1; //SUB LSB 2 - Bit02
    //ui8SerialData[30] = 0; //SUB LSB 1 - Bit01
    //ui8SerialData[31] = 0; //SUB LSB 0 - Bit00

    if (ui32RawDataValue > MAX_ADC_VALUE) {
        ui32RawDataValue = MAX_ADC_VALUE;
    }

    return ui32RawDataValue;
}

float
get_voltage(uint32_t ui32RawDataValue)
{
    float fVoltage;

    fVoltage = ui32RawDataValue;
    fVoltage = (fVoltage * VOLTAGE_REF) / (MAX_ADC_VALUE+1);
    fVoltage = fVoltage * 10;

    if (fVoltage > MAX_VOLTAGE_VALUE) {
        fVoltage = 50;
    }

    if (fVoltage < MIN_VOLTAGE_VALUE) {
        fVoltage = 0;
    }

    return fVoltage;
}

void
set_adc_chip_select(uint8_t chipSelectState)
{
    switch(chipSelectState) {
    case ADC_CS_LOW:
        // Make pin 4 low
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
        break;
    case ADC_CS_HIGH:
        // Make pin 4 high
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, GPIO_PIN_4);
        break;
    default:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);
    }
}

void
set_debug_output(uint8_t debugOutputState)
{
    switch(debugOutputState) {
    case DEBUG_LOW:
        // Make pin 5 low
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
        break;
    case DEBUG_HIGH:
        // Make pin 5 high
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
        break;
    default:
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
    }
}

void
release_user_input_interrupt_triggers(void)
{
    static uint16_t counter = 0;

    if (counter > USER_INPUT_INT_TRIGGER_COUNTER_LIMIT) {
        g_ui32UserInputIntTrigger = true;
        counter = 0;
    } else {
        counter++;
    }
}

void
get_user_input(void)
{
    release_user_input_interrupt_triggers();

    if (g_ui32CalibrationButtonPressed == true) {
        //if calibration button has been pressed it means the user wants to
        //store the offset zero value and store it in the EEPROM. Once the
        //offset value has been stored in the EEPROM it can be retrieved on
        //a hardware startup and this value can be used to minus the
        //offset value from the raw input voltage.
        //e.g. InputVoltage - offsetValue = OutputVoltage
        switch(g_ui8DebugOutputState) {
        case DEBUG_LOW:
            set_debug_output(DEBUG_LOW);
            g_ui8DebugOutputState = DEBUG_HIGH;
            break;
        case DEBUG_HIGH:
            set_debug_output(DEBUG_HIGH);
            g_ui8DebugOutputState = DEBUG_LOW;
            break;
        default:
            g_ui8DebugOutputState = DEBUG_LOW;
        }
        g_ui32CalibrationButtonPressed = false;
    }

}

//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int
main(void)
{
    //Initialize local variables
    uint32_t ui32RawDataValue;
    float fVoltage;
    char outputBuffer[OUTPUT_BUFFER_LENGTH];
    uint8_t outputPulseHighFlag = true;
    uint32_t outputPulseCounter = 0;

    //Initialize global variables
    g_ui32SerialDataState = SERIAL_DATA_LOW;
    g_ui32SerialDataCounter = 0;
    g_ui32UserInputIntTrigger = true;
    g_ui32CalibrationButtonPressed = false;

    // Configs
    config_system_clock();
    config_fpu();
    config_interrupts();
    configure_uart();
    configure_adc_chip_select();
    configure_debug();



    while(1)
    {
        //this triggers the ADC
        if(outputPulseHighFlag == true) {
            //Setting CS low will trigger single ADC operation
            set_adc_chip_select(ADC_CS_LOW);
            ms_delay(3);
            set_adc_chip_select(ADC_CS_HIGH);
            outputPulseHighFlag = false;
        }

        if (g_ui32SerialDataCounter >= SERIAL_DATA_BUFFER_LENGTH) {
            ui32RawDataValue = get_adcvalue();
            fVoltage = get_voltage(ui32RawDataValue);
            sprintf(outputBuffer, "Voltage: %f", fVoltage);
            UARTprintf("%s\n", outputBuffer);
            memset(outputBuffer, 0, OUTPUT_BUFFER_LENGTH);
            ui32RawDataValue = 0;
            fVoltage = 0;
        }

        if(outputPulseCounter > 500) {
            outputPulseHighFlag = true;
            outputPulseCounter = 0;
        } else {
            outputPulseCounter++;
        }

        get_user_input();

        ms_delay(10);

    }

}
