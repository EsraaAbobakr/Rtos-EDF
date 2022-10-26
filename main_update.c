/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"
#include "queue.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */


TaskHandle_t Button_1_Monitor_Handler = NULL;
TaskHandle_t Button_2_Monitor_Handler = NULL;
TaskHandle_t Periodic_Transmitter_Handler = NULL;
TaskHandle_t Uart_Receiver_Handler = NULL;
TaskHandle_t Load_1_Simulation_Handler = NULL;
TaskHandle_t Load_2_Simulation_Handler = NULL;
QueueHandle_t xQueue1=NULL;
QueueHandle_t xQueue2=NULL;
QueueHandle_t xQueue3=NULL;
static void prvSetupHardware( void );
/*-----------------------------------------------------------*/
void Button_1_Monitor ( void * pvParameters )
{/*Button _1 port 0 pin o*/
	pinState_t Previous_State=GPIO_read(PORT_1,PIN14);
	pinState_t Current_State;
	char * Falling_String = "Falling_Button1";
	char * Rising_String = "Risiing_Button1";
	uint16_t i,j;
	TickType_t xLastWakeTime = xTaskGetTickCount();
    for( ;; )
			{		Current_State=GPIO_read(PORT_1,PIN14);
				
				if(Current_State==PIN_IS_LOW && Previous_State==PIN_IS_HIGH)
				{/*falling edge*/
					for(i = 0 ; i < 16 ; i++)
					{
        	 xQueueSend( xQueue1,( void * )&Rising_String[i],( TickType_t ) 0) ;
					}
				}
				if (Current_State==PIN_IS_HIGH && Previous_State==PIN_IS_LOW)
				{/*rising edge*/
					for(j = 0 ; j < 16 ; j++)
					{
        	 xQueueSend( xQueue1,( void * )&Falling_String[j],( TickType_t ) 0) ;
					}
				}
				Previous_State=Current_State;
				
			vTaskDelayUntil( &xLastWakeTime , 50);
			GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
			}
}
void Button_2_Monitor ( void * pvParameters )
{/*Button _1 port 1 pin o*/
	pinState_t Previous_State=GPIO_read(PORT_1,PIN15);
	pinState_t Current_State;
	char * Falling_String = "Falling_Button2";
	char * Rising_String = "Risiing_Button2";
	uint16_t i,j;
	TickType_t xLastWakeTime = xTaskGetTickCount();
    for( ;; )
			{		Current_State=GPIO_read(PORT_1,PIN15);
				
				if(Current_State==PIN_IS_LOW && Previous_State==PIN_IS_HIGH)
				{/*falling edge*/
					for(i = 0 ; i < 16 ; i++)
					{
        	 xQueueSend( xQueue2,( void * )&Rising_String[i],( TickType_t ) 10) ;
					}
				}
				if (Current_State==PIN_IS_HIGH && Previous_State==PIN_IS_LOW)
				{/*rising edge*/
					for(j = 0 ; j < 16 ; j++)
					{
        	 xQueueSend( xQueue2,( void * )&Falling_String[j],( TickType_t ) 10) ;
					}
				}
				Previous_State=Current_State;
				
			vTaskDelayUntil( &xLastWakeTime , 50);
				GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
			}
}
void Periodic_Transmitter ( void * pvParameters )
{
	TickType_t xLastWakeTime = xTaskGetTickCount();
	uint16_t k;
//	char * Periodic_String ="Periodic String";
    for( ;; )
			{		char * Periodic_String ="Periodic String";
				for(k = 0 ; k < 16 ; k++)
					{if(xQueue3 != 0)
        	 xQueueSend( xQueue3,( void * )&Periodic_String[k],( TickType_t ) 0) ;
					}
				vTaskDelayUntil( &xLastWakeTime , 100);
					GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
			}
}
//void Uart_Receiver ( void * pvParameters )
//{
//	TickType_t xLastWakeTime = xTaskGetTickCount();
//	const signed char * Receiver;
//	char receiverBuffer;
//	uint16_t l;
//    for( ;; )
//			{		
//				 if( xQueue1 != NULL )
//				 {
//							for( l = 0 ; l < 16; l++)
//						{
//							xQueueReceive( xQueue1, ( void * ) (Receiver+l),  0 );
//						}
//					if ((xQueueReceive(xQueue1,(void *) &(receiverBuffer), (TickType_t)0)) == pdPASS )
//						vSerialPutString(Receiver,20);	
//				 }
//   		vTaskDelayUntil( &xLastWakeTime , 20);
//				 GPIO_write(PORT_0,PIN2,PIN_IS_LOW);

//			}
//}

void Load_1_Simulation ( void * pvParameters )
{/*5ms*/
	 TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t  i=0;
	uint32_t  X=60000;
    for( ;; )
			{		
				GPIO_write(PORT_1,PIN1,PIN_IS_HIGH);
				for (i =0 ; i<X;i++)
				{}
				GPIO_write(PORT_1,PIN1,PIN_IS_LOW);
   		  vTaskDelayUntil( &xLastWakeTime , 10);
				GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
			}
}
void Load_2_Simulation ( void * pvParameters )
{/*12ms*/
	 TickType_t xLastWakeTime = xTaskGetTickCount();
	uint32_t  i=0;
	uint32_t  X=150000;
    for( ;; )
			{		
				GPIO_write(PORT_1,PIN2,PIN_IS_HIGH);
				for (i =0 ; i<X;i++)
				{}
				GPIO_write(PORT_1,PIN2,PIN_IS_LOW);
   		  vTaskDelayUntil( &xLastWakeTime , 100);
				GPIO_write(PORT_0,PIN2,PIN_IS_LOW);

			}
}
/*implement tick hook*/
void vApplicationTickHook (void)
{
	GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
	GPIO_write(PORT_0,PIN1,PIN_IS_LOW);
	//GPIO_write(PORT_0,PIN1,PIN_IS_HIGH);
}
/*implement idle hook*/
void vApplicationIdleHook (void)
{
	GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
	//GPIO_write(PORT_0,PIN2,PIN_IS_LOW);
	//GPIO_write(PORT_0,PIN2,PIN_IS_HIGH);
}







/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */
int main( void )

{
	/* Setup the hardware for use with the Keil demo board. */
	prvSetupHardware();

	/* Create a queue capable of containing 10 unsigned long values. */
    xQueue1 = xQueueCreate( 10, sizeof( char ) );
	  xQueue2 = xQueueCreate( 10, sizeof( char ) );
	  xQueue3 = xQueueCreate( 10, sizeof( char ) );
    /* Create Tasks here */
	xTaskPeriodicCreate(
                    Button_1_Monitor,       /* Function that implements the task. */
                    "Button_1",          /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_1_Monitor_Handler,50 );      /* Used to pass out the created task's handle. */
										
	 xTaskPeriodicCreate(
                    Button_2_Monitor,       /* Function that implements the task. */
                    "Button_2",          /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Button_2_Monitor_Handler,50 );      /* Used to pass out the created task's handle. */
		xTaskPeriodicCreate(
                    Periodic_Transmitter,       /* Function that implements the task. */
                    "Transmitter",          /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Periodic_Transmitter_Handler ,100);      /* Used to pass out the created task's handle. */
//										
//	 xTaskPeriodicCreate(
//                    Uart_Receiver,       /* Function that implements the task. */
//                    "Receiver",          /* Text name for the task. */
//                    configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
//                    ( void * ) 0,    /* Parameter passed into the task. */
//                    1,/* Priority at which the task is created. */
//                    &Uart_Receiver_Handler,20 );      /* Used to pass out the created task's handle. */
	 xTaskPeriodicCreate(
                    Load_1_Simulation,       /* Function that implements the task. */
                    "Load_1",          /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_1_Simulation_Handler,10 );      /* Used to pass out the created task's handle. */
	 xTaskPeriodicCreate(
                    Load_2_Simulation,       /* Function that implements the task. */
                    "Load_2",          /* Text name for the task. */
                    configMINIMAL_STACK_SIZE,      /* Stack size in words, not bytes. */
                    ( void * ) 0,    /* Parameter passed into the task. */
                    1,/* Priority at which the task is created. */
                    &Load_2_Simulation_Handler,100 );      /* Used to pass out the created task's handle. */

 

	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;
}
/*-----------------------------------------------------------*/

