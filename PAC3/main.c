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
#include "edu_boosterpack_buttons.h"




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

// callbacks & functions
void callback(adc_result input);
void buttonCallback(void);
const char* getMove(int play);
void printString(char str[], bool newLine);
void restartGame();
enum message_code getMessageWinner(void);
//Task sync tools and variables
SemaphoreHandle_t xButtonPressed;   //semáforo para activar la tarea ProcessingTask cuando se pulsa S1
QueueHandle_t xQueueCommands;       //cola para que tanto la tarea ADCReadingTask como ProcessingTask envien comandos de tipo message_code a la tarea UARTPrintingTask
QueueHandle_t xQueueADC;



typedef enum message_code{
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
play machine_play = NULL;
bool firstInitialization = true;
bool ignoreNextReading = false;
TickType_t xTicks;
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
    char toPrint[50];
    message_code message;
    for(;;){
        if (firstInitialization == true) {
            sprintf(toPrint, "Introduce tu jugada: %s", getMove(my_play));
            printString(toPrint, false);
            firstInitialization = false;
        }

        if( xQueueReceive( xQueueCommands, &message, portMAX_DELAY ) == pdPASS){
            if (message == play_update_message) {
                sprintf(toPrint, "Introduce tu jugada: %s", getMove(my_play));
                printString(toPrint, false);
            } else {
                sprintf(toPrint, "La maquina ha escogido: %s", getMove(machine_play));
                printString(toPrint, true);
                if (message == i_win_message) {
                    sprintf(toPrint, "Tu ganas!");
                    printString(toPrint, true);
                } else if (message == machine_wins_message) {
                    sprintf(toPrint, "Tu pierdes!");
                    printString(toPrint, true);
                }else if (message == tie_message) {
                    sprintf(toPrint, "Empate!");
                    printString(toPrint, true);
                }
                restartGame();
            }
        }

        vTaskDelay( pdMS_TO_TICKS(DELAY_MS) );
    }
}

void restartGame() {
    char toPrint[50];
    sprintf(toPrint, "\n\r");
    printString(toPrint, true);

    firstInitialization = true;
    ignoreNextReading = false;
}
void printString(char str[], bool newLine) {
    char message[50];

    if (!newLine) {
        sprintf(message, "\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b");
        uart_print(message);
    } else {
        sprintf(message, "\n\r");
        uart_print(message);
    }

    sprintf(message, str);
    uart_print(message);
}

const char* getMove(int play) {
    if (play == rock) {
        return "PIEDRA ";
    }

    if (play == scissors) {
        return "TIJERAS";
    }

    if (play == paper) {
        return "PAPEL  ";
    }

    return 'Unkown move';
}

static void ProcessingTask(void *pvParameters) {
    message_code winner;

    for(;;){
        if (xSemaphoreTake(xButtonPressed, portMAX_DELAY) == pdPASS) {
            if (xTicks == NULL || (xTaskGetTickCount()-xTicks) > pdMS_TO_TICKS(DELAY_MS)) {
                xTicks = xTaskGetTickCount();
                int randVal = rand() % 3;
                if (randVal == 0) {
                    machine_play = rock;
                }else if (randVal == 1) {
                    machine_play = paper;
                } else {
                    machine_play = scissors;
                }

                winner = getMessageWinner();
                xQueueSendFromISR(xQueueCommands, &winner, NULL);
            }


        }
        vTaskDelay( pdMS_TO_TICKS(DELAY_MS) );
    }
}

enum message_code getMessageWinner(void) {
    if (my_play == machine_play) {
        return tie_message;
    } else {
        if (my_play == rock) {
            if (machine_play == scissors) {
                return i_win_message;
            }
        }else if(my_play == paper) {
            if (machine_play == rock) {
                return i_win_message;
            }
        }else if(my_play == scissors) {
            if (machine_play == paper) {
                return i_win_message;
            }
        }
    }

    return machine_wins_message;
}

void callback(adc_result input) {
    float x = input[0];
    float y = input[1];

    xQueueSendFromISR(xQueueADC, &y, NULL);
}

void buttonCallback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xButtonPressed,  xHigherPriorityTaskWoken);
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
    edu_boosterpack_buttons_init();
    edu_boosterpack_buttons_set_callback(MSP432_EDU_BOOSTERPACK_BUTTON_S1, buttonCallback);

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
