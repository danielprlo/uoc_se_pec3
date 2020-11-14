/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/

/* Standard includes */
#include <stdlib.h>
#include <stdio.h>


/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "portmacro.h"


/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"
#include "uart_driver.h"
#include "edu_boosterpack_joystick.h"




/*----------------------------------------------------------------------------*/

#define TASK_PRIORITY               ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define TASK_STACK_SIZE             ( 1024 )
#define HEARTBEAT_STACK_SIZE        ( 128 )

#define HEART_BEAT_ON_MS            ( 10 )
#define HEART_BEAT_OFF_MS           ( 990 )
#define DELAY_MS                    ( 100 )


#define QUEUE_SIZE                  ( 10 )
#define TX_UART_MESSAGE_LENGTH      ( 80 )


/*----------------------------------------------------------------------------*/



// Tasks
static void HeartBeatTask(void *pvParameters);
static void ADCReadingTask(void *pvParameters);
static void UARTPrintingTask(void *pvParameters);
static void ProcessingTask(void *pvParameters);
static void PrintPlay(int newPlay);
// callbacks & functions
void callback(adc_result input);


//Task sync tools and variables
SemaphoreHandle_t xButtonPressed;   //semáforo para activar la tarea ProcessingTask cuando se pulsa S1
QueueHandle_t xQueueCommands;       //cola para que tanto la tarea ADCReadingTask como ProcessingTask envien comandos de tipo message_code a la tarea UARTPrintingTask
QueueHandle_t xQueueADC;



typedef enum{
    play_update_message = 0,
    i_win_message = 1,
    machine_wins_message = 2,
    tie_message = 3,
} message_code;

typedef enum{
    rock = 0,
    paper = 1,
    scissors = 2,
}play;

play my_play = paper;    //variables globales que contienen la jugada del usuario (my_play) y la jugada de la máquina (machine_play)
play machine_play;
bool firstInitialization = true;
bool ignoreNextReading = false;
/*----------------------------------------------------------------------------*/

static void HeartBeatTask(void *pvParameters){
    for(;;){
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_ON_MS) );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( pdMS_TO_TICKS(HEART_BEAT_OFF_MS) );
    }
}


static void ADCReadingTask(void *pvParameters) {
    float y;
    for(;;){
        if (!ignoreNextReading) {
            edu_boosterpack_joystick_read();

            if( xQueueReceive( xQueueADC, &y, portMAX_DELAY ) == pdPASS){
                //Right choice
               if (y > 13000) {
                   int newPlay = (((int)my_play+1) > 2) ? 0 : (int)my_play+1;
                   my_play = newPlay;
                   //PrintPlay(newPlay);
                   ignoreNextReading = true;
                   message_code message = play_update_message;
                   xQueueSendFromISR(xQueueCommands, &message, NULL);
               }

               //Left choice
               if (y < 3000) {
                   int newPlay = (((int)my_play-1) < 0) ? 2 : (int)my_play-1;
                   my_play = newPlay;
                   //PrintPlay(newPlay);
                   ignoreNextReading = true;
                   message_code message = play_update_message;
                   xQueueSendFromISR(xQueueCommands, &message, NULL);
               }

            }
        } else {
            ignoreNextReading = false;
        }


        vTaskDelay( pdMS_TO_TICKS(DELAY_MS) );
    }
}

static void UARTPrintingTask(void *pvParameters) {

    message_code message;
    for(;;){
        if (firstInitialization == true) {
            PrintPlay(my_play);
            firstInitialization = false;
        }

        if( xQueueReceive( xQueueCommands, &message, portMAX_DELAY ) == pdPASS){
            if (message == play_update_message) {
                PrintPlay(my_play);
            }
        }

        vTaskDelay( pdMS_TO_TICKS(DELAY_MS) );
    }
}

static void PrintPlay(int newPlay) {
    char message[50];
    char *play;

    if (newPlay == rock) {
        play = "PIEDRA ";
    }

    if (newPlay == scissors) {
        play = "TIJERAS";
    }

    if (newPlay == paper) {
        play = "PAPEL  ";
    }

    sprintf(message, "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b", play);
    uart_print(message);
    sprintf(message, "Introduce tu jugada: %s", play);
    uart_print(message);

}

static void ProcessingTask(void *pvParameters) {

    for(;;){
        vTaskDelay( pdMS_TO_TICKS(DELAY_MS) );
    }
}

void callback(adc_result input) {
    float x = input[0];
    float y = input[1];

    xQueueSendFromISR(xQueueADC, &y, NULL);
}

/*----------------------------------------------------------------------------*/

int main(int argc, char** argv)
{
    int32_t retVal = -1;

    // Initialize semaphores and queue
    xButtonPressed = xSemaphoreCreateBinary ();
    xQueueCommands = xQueueCreate( QUEUE_SIZE, sizeof( message_code ) );
    xQueueADC = xQueueCreate( QUEUE_SIZE, sizeof( adc_result ) );

    /* Initialize the board */
    board_init();

    /* Initialize the UART */  //configurada para trabajar a 57600bauds/s
    uart_init(NULL);

    /* Initialize the button */


    /* Initialize the joystick*/
    edu_boosterpack_joystick_init();
    edu_boosterpack_joystick_set_callback(callback);


    if ( (xButtonPressed != NULL) && (xQueueCommands != NULL) && (xQueueADC != NULL)) {

        /* Create tasks */
        retVal = xTaskCreate(HeartBeatTask, "HeartBeatTask", HEARTBEAT_STACK_SIZE, NULL, HEARTBEAT_TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        retVal = xTaskCreate(ADCReadingTask, "ADCReadingTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        retVal = xTaskCreate(UARTPrintingTask, "UARTPrintingTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        retVal = xTaskCreate(ProcessingTask, "ProcessingTask", TASK_STACK_SIZE, NULL, TASK_PRIORITY, NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }


        /* Start the task scheduler */
        vTaskStartScheduler();
    }

    return 0;
}

/*----------------------------------------------------------------------------*/
