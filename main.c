//*****************************************************************************
//
// blinky.c - Simple example to blink the on-board LED.
//
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"

#define LED_RED     GPIO_PIN_1
#define LED_BLUE    GPIO_PIN_2
#define LED_GREEN   GPIO_PIN_3

#define LED_OFF     0
#define LED_ON      1


volatile uint32_t g_ui32LedStatus;



//*****************************************************************************
//
// This is the handler for INT_GPIOA.  It simply saves the interrupt sequence
// number.
//
//*****************************************************************************
void
PortAIntHandler(void)
{

    GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2);

    switch(g_ui32LedStatus) {

    case LED_OFF:
        //
        // Turn off the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, 0x0);
        g_ui32LedStatus = LED_ON;
        break;

    case LED_ON:
        //
        // Turn on the LED.
        //
        GPIOPinWrite(GPIO_PORTF_BASE, LED_BLUE, LED_BLUE);
        g_ui32LedStatus = LED_OFF;
        break;
    }

}


//*****************************************************************************
//
// Configure the led
//
//*****************************************************************************
void
config_led(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_BLUE);
}

void
config_interrupts(void){

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
    GPIOIntRegister(GPIO_PORTA_BASE, PortAIntHandler);

    //
    // Initialize the GPIO pin configuration.
    // Set pin 2 as input, SW controlled.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Make pins 2 and 4 rising edge triggered interrupts.
    //
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);

    //
    // Enable the pin interrupts.
    //
    GPIOIntEnable(GPIO_PORTA_BASE, GPIO_PIN_2);

    //
    // Register the interrupt handler function for UART 0.
    //
    IntRegister(INT_GPIOA, PortAIntHandler);
    //
    // Enable the interrupt for UART 0.
    //
    IntEnable(INT_GPIOA);
    //
    // Enable UART 0.
    //
    IntMasterEnable();

}


//*****************************************************************************
//
// Blink the on-board LED.
//
//*****************************************************************************
int
main(void)
{
    volatile uint32_t ui32Loop;

    //
    // Set the clocking to run directly from the crystal.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

    g_ui32LedStatus = LED_ON;


    // Configs
    config_led();
    config_interrupts();


    while(1)
    {
        //
        // Loop forever.
        //
    }

}
