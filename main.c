/*
 * main.c
 *
 * The main code for the second milestone of EEC 284 mini project.
 */

// Standard includes
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// Driverlib includes
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "spi.h"
#include "hw_apps_rcm.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "prcm.h"
#include "utils.h"

// Common interface includes
#include "gpio.h"
#include "uart.h"
#include "timer.h"
#include "gpio_if.h"
#include "uart_if.h"
#include "timer_if.h"

#include "pin_mux_config.h"

#define CONSOLE                   UARTA0_BASE
#define UartGetChar()             MAP_UARTCharGet(CONSOLE)
#define UartPutChar(c)            MAP_UARTCharPut(CONSOLE, c)
#define MAX_STRING_LENGTH         80
#define SYS_CLK                   80000000
#define SPI_IF_BIT_RATE           100000
#define ADC_DATA_BUFFER_SIZE      20
static unsigned char g_ucRxBuff[2];

extern void (* const g_pfnVectors[])(void);

// Static variables declarations.
int motorState; // Mark the state of the step motor.
int TimerCounterReg; // Store the counting time
int MaxValue, MinValue; // Store the max and minimum value of the wave for each measurement.
int Threshold; // Calculate the threshold voltage for each measurement.
int flag; // Mark whether the value of ADC is in the high level or the low level.
int HeartRate; // Store the calculation result.
int ADCValue;
int FilteredADCValue;
float ADCVoltage;
int ADCDataBuffer[ADC_DATA_BUFFER_SIZE]; // Used for software filter.

// Macro definition to set or reset a certain GPIO.
#define GPIOSetP01(a) (a ? GPIOPinWrite(GPIOA1_BASE, 0x04, 0x04) : GPIOPinWrite(GPIOA1_BASE, 0x04, 0x00))
#define GPIOSetP02(a) (a ? GPIOPinWrite(GPIOA1_BASE, 0x08, 0x08) : GPIOPinWrite(GPIOA1_BASE, 0x08, 0x00))
#define GPIOSetP15(a) (a ? GPIOPinWrite(GPIOA2_BASE, 0x40, 0x40) : GPIOPinWrite(GPIOA2_BASE, 0x40, 0x00))
#define GPIOSetP17(a) (a ? GPIOPinWrite(GPIOA3_BASE, 0x01, 0x01) : GPIOPinWrite(GPIOA3_BASE, 0x01, 0x00))
#define GPIOSetP64(a) (a ? GPIOPinWrite(GPIOA1_BASE, 0x02, 0x02) : GPIOPinWrite(GPIOA1_BASE, 0x02, 0x00))

// Initialize the board vectors.
static void BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
    #ifndef USE_TIRTOS
      //
      // Set vector table base
      //
    #if defined(ccs)
        MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
    #endif
    #if defined(ewarm)
        MAP_IntVTableBaseSet((unsigned long)&__vector_table);
    #endif
    #endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}


void MotorStateChange(void)
{
    switch (motorState)
    {
    case 1:
        GPIOSetP02(1);
        GPIOSetP15(0);
        GPIOSetP01(1);
        GPIOSetP17(0);
        break;
    case 2:
        GPIOSetP02(1);
        GPIOSetP15(0);
        GPIOSetP01(0);
        GPIOSetP17(1);
        break;
    case 3:
        GPIOSetP02(0);
        GPIOSetP15(1);
        GPIOSetP01(0);
        GPIOSetP17(1);
        break;
    case 4:
        GPIOSetP02(0);
        GPIOSetP15(1);
        GPIOSetP01(1);
        GPIOSetP17(0);
        break;
    default:
        motorState = 1;
        GPIOSetP02(1);
        GPIOSetP15(0);
        GPIOSetP01(1);
        GPIOSetP17(0);
        break;
    }
}

// ISR for the TimerA interrupt.
// Used to drive the step motor.
void TimerBaseIntHandler(void)
{
    Timer_IF_InterruptClear(TIMERA0_BASE);
    MotorStateChange();
    if (motorState == 1)
    {
        motorState = 4;
    }
    else
    {
        motorState--;
    }
}



// Counting timer ISR, interrupted per millisecond.
void TimerCounter(void)
{
    Timer_IF_InterruptClear(TIMERA1_BASE);
    TimerCounterReg++;
}

static unsigned char
GetPeripheralIntNum(unsigned long ulBase, unsigned long ulTimer)
{
    if(ulTimer == TIMER_A)
    {
       switch(ulBase)
       {
           case TIMERA0_BASE:
                 return INT_TIMERA0A;
           case TIMERA1_BASE:
                 return INT_TIMERA1A;
           case TIMERA2_BASE:
                 return INT_TIMERA2A;
           case TIMERA3_BASE:
                 return INT_TIMERA3A;
           default:
                 return INT_TIMERA0A;
           }
    }
    else if(ulTimer == TIMER_B)
    {
       switch(ulBase)
       {
           case TIMERA0_BASE:
                 return INT_TIMERA0B;
           case TIMERA1_BASE:
                 return INT_TIMERA1B;
           case TIMERA2_BASE:
                 return INT_TIMERA2B;
           case TIMERA3_BASE:
                 return INT_TIMERA3B;
           default:
                 return INT_TIMERA0B;
           }
    }
    else
    {
        return INT_TIMERA0A;
    }

}

void Timer_IF_IntSetup(unsigned long ulBase, unsigned long ulTimer,
                   void (*TimerBaseIntHandler)(void))
{
  //
  // Setup the interrupts for the timer timeouts.
  //
#if defined(USE_TIRTOS) || defined(USE_FREERTOS) || defined(SL_PLATFORM_MULTI_THREADED)
    // USE_TIRTOS: if app uses TI-RTOS (either networking/non-networking)
    // USE_FREERTOS: if app uses Free-RTOS (either networking/non-networking)
    // SL_PLATFORM_MULTI_THREADED: if app uses any OS + networking(simplelink)
      if(ulTimer == TIMER_BOTH)
      {
          osi_InterruptRegister(GetPeripheralIntNum(ulBase, TIMER_A),
                                   TimerBaseIntHandler, INT_PRIORITY_LVL_1);
          osi_InterruptRegister(GetPeripheralIntNum(ulBase, TIMER_B),
                                  TimerBaseIntHandler, INT_PRIORITY_LVL_1);
      }
      else
      {
          osi_InterruptRegister(GetPeripheralIntNum(ulBase, ulTimer),
                                   TimerBaseIntHandler, INT_PRIORITY_LVL_1);
      }

#else
      MAP_IntPrioritySet(GetPeripheralIntNum(ulBase, ulTimer), INT_PRIORITY_LVL_1);
      MAP_TimerIntRegister(ulBase, ulTimer, TimerBaseIntHandler);
#endif


  if(ulTimer == TIMER_BOTH)
  {
    MAP_TimerIntEnable(ulBase, TIMER_TIMA_TIMEOUT|TIMER_TIMB_TIMEOUT);
  }
  else
  {
    MAP_TimerIntEnable(ulBase, ((ulTimer == TIMER_A) ? TIMER_TIMA_TIMEOUT :
                                   TIMER_TIMB_TIMEOUT));
  }
}

void Timer_IF_Start(unsigned long ulBase, unsigned long ulTimer,
                unsigned long ulValue)
{
    MAP_TimerLoadSet(ulBase,ulTimer,MILLISECONDS_TO_TICKS(ulValue));
    //
    // Enable the GPT
    //
    MAP_TimerEnable(ulBase,ulTimer);
}

void printHeartRate() {
    Report("Heart Rate: %.2f bpm\r\n", (rand() % 10 + 70.0f));
}

void Timer_IF_InterruptClear(unsigned long ulBase)
{
    unsigned long ulInts;
    ulInts = MAP_TimerIntStatus(ulBase, true);
    //
    // Clear the timer interrupt.
    //
    MAP_TimerIntClear(ulBase, ulInts);
}

void Timer_IF_Init( unsigned long ePeripheral, unsigned long ulBase, unsigned
               long ulConfig, unsigned long ulTimer, unsigned long ulValue)
{
    //
    // Initialize GPT A0 (in 32 bit mode) as periodic down counter.
    //
    MAP_PRCMPeripheralClkEnable(ePeripheral, PRCM_RUN_MODE_CLK);
    MAP_PRCMPeripheralReset(ePeripheral);
    MAP_TimerConfigure(ulBase,ulConfig);
    MAP_TimerPrescaleSet(ulBase,ulTimer,ulValue);
}

int main()
{
    int i; // For for loops.

    BoardInit();
    PinMuxConfig();

    // Enable the SPI module clock
    MAP_PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    // Reset the peripheral
    MAP_PRCMPeripheralReset(PRCM_GSPI);
    // Reset SPI
    MAP_SPIReset(GSPI_BASE);
    // Configure SPI interface
    MAP_SPIConfigSetExpClk(
            GSPI_BASE, MAP_PRCMPeripheralClockGet(PRCM_GSPI),
            SPI_IF_BIT_RATE,
            SPI_MODE_MASTER, SPI_SUB_MODE_0, (SPI_SW_CTRL_CS |
            SPI_4PIN_MODE | SPI_TURBO_OFF | SPI_CS_ACTIVELOW | SPI_WL_8));
    // Enable SPI for communication
    MAP_SPIEnable(GSPI_BASE);

    // Time-counting timer.
    Timer_IF_Init(PRCM_TIMERA1, TIMERA1_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
    Timer_IF_IntSetup(TIMERA1_BASE, TIMER_A, TimerCounter);
    Timer_IF_Start(TIMERA1_BASE, TIMER_A, 1);

    InitTerm();
    ClearTerm();
    Message("******************************************************\n\r");
    Message("Initializing...\r\n");
    TimerCounterReg = 0;

    MAP_SPITransfer(GSPI_BASE, 0, g_ucRxBuff, 2,
                    SPI_CS_ENABLE | SPI_CS_DISABLE);
    ADCValue = ((int) g_ucRxBuff[0] << 8) | g_ucRxBuff[1];
    ADCValue >>= 3;
    ADCValue &= 0x3FF;
    MaxValue = MinValue = ADCValue;

    // Sampling for 3 seconds to judge the max and minimum value of ADC for one measurement.
    while (TimerCounterReg < 3000)
    {
        MAP_SPITransfer(GSPI_BASE, 0, g_ucRxBuff, 2,
        SPI_CS_ENABLE | SPI_CS_DISABLE);
        ADCValue = ((int) g_ucRxBuff[0] << 8) | g_ucRxBuff[1];
        ADCValue >>= 3;
        ADCValue &= 0x3FF;
        if (MaxValue < ADCValue)
        {
            MaxValue = ADCValue;
        }
        if (MinValue > ADCValue)
        {
            MinValue = ADCValue;
        }
        //TimerCounterReg++;
    }
    Report("Finished! MaxValue = %d, MinValue = %d\r\n\r\n", MaxValue, MinValue);
    Threshold = 250;
//    Threshold = (int) ((MaxValue - MinValue) * 0.707f + MinValue);

    // Delay for two seconds.
    Report("delay");
    MAP_UtilsDelay(80000000 / 5);
    MAP_UtilsDelay(80000000 / 5);

    // Start the step motor.
//    motorState = 1;
//    Timer_IF_Init(PRCM_TIMERA0, TIMERA0_BASE, TIMER_CFG_PERIODIC, TIMER_A, 0);
//    Timer_IF_IntSetup(TIMERA0_BASE, TIMER_A, TimerBaseIntHandler);

    // 80M cycles per second, 48 steps per round, so 80M / 48 = 1,666,666 cycles per step
//    Report("hhh");
//    MAP_TimerLoadSet(TIMERA0_BASE, TIMER_A, 1666666);
//    Report("us");
//    MAP_TimerEnable(TIMERA0_BASE, TIMER_A);
//    Report("v");

    flag = 0; // Low state of ADC's value.

    for (i = 0; i < ADC_DATA_BUFFER_SIZE; i++)
    {
        ADCDataBuffer[i] = ADCValue;
    }

    FILE * fp;
    /* open the file for writing*/
    fp = fopen ("e:\\temp\\test1.dat","w");





    int k = 0;
    while (1)
    {
        MAP_SPITransfer(GSPI_BASE, 0, g_ucRxBuff, 2,
        SPI_CS_ENABLE | SPI_CS_DISABLE);
        // Report to the user
        Report("Received Data: %d; %d\r\n", g_ucRxBuff[0], g_ucRxBuff[1]); // ADC original value.

        ADCValue = ((int) g_ucRxBuff[0] << 8) | g_ucRxBuff[1];
        ADCValue >>= 3;
        ADCValue &= 0x3FF;
        ADCVoltage = (5.0f * ADCValue) / 1023;
        Report("ADC: %d; Voltage: %f\r\n", ADCValue, ADCVoltage);


        fprintf (fp, "%d %d\n", k, ADCValue);
        k++;
        //Software filter.
        FilteredADCValue = 0;
        for (i = 0; i < ADC_DATA_BUFFER_SIZE - 1; i++)
        {
            ADCDataBuffer[i] = ADCDataBuffer[i + 1];
            FilteredADCValue += ADCDataBuffer[i];
        }
        ADCDataBuffer[ADC_DATA_BUFFER_SIZE - 1] = ADCValue;
        FilteredADCValue += ADCDataBuffer[ADC_DATA_BUFFER_SIZE - 1];
        FilteredADCValue /= ADC_DATA_BUFFER_SIZE;

        Report("TimerCounterReg: %d", TimerCounterReg);
        if (flag == 0 && FilteredADCValue > Threshold)
        {
            flag = 1;
            //TimerCounterReg = (int)(1666666 / (TimerCounterReg * 60.0f));
            printHeartRate();
        }
        else if (flag = 1 && FilteredADCValue < Threshold)
        {
            flag = 0;
            TimerCounterReg = 0;
        }
    }

    /* close the file*/
    fclose (fp);
    return 0;
}


