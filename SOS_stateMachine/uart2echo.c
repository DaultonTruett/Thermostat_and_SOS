/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== uart2echo.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// Struct for various states the state machine can be in - determines how and when to turn red LED on or off - DT
struct smStates{
    unsigned char START;
    unsigned char OFF;
    unsigned char ON;
    unsigned char O;
    unsigned char F1;
};


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    // User input
    char input;
    // State variable will be reassigned based on user input and will determine if red LED is ON or OFF - DT
    unsigned char state;

    const char echoPrompt[] = "Echoing characters:\r\n";
    UART2_Handle uart;
    UART2_Params uartParams;
    size_t bytesRead;
    size_t bytesWritten = 0;
    uint32_t status     = UART2_STATUS_SUCCESS;

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED pin */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    /* Create a UART where the default read and write mode is BLOCKING */
    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART2_open() failed */
        while (1) {}
    }

    /* Turn on user LED to indicate successful initialization */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    UART2_write(uart, echoPrompt, sizeof(echoPrompt), &bytesWritten);


    // Create the sm variable from the smStates struct to keep track of current state - DT
    struct smStates sm;

    // Init the sm variables - DT
    sm.START = '0';
    sm.OFF = '1';
    sm.ON = '2';
    sm.O = '3';
    sm.F1 = '4';

    // Init 'state' as the sm struct variable 'START':
    // Will be used as the starting state and as a return state after the red LED has been turned ON, OFF, or incorrect characters are input - DT
    state = sm.START;

    // Enter the state machine -  DT
    while(1){

        UART2_read(uart, &input, 1, &bytesRead);

        // Input is O and current state is START
        if((input == 'O') && (state == sm.START)){
            // Move state to O
            state = sm.O;

         // Input is N and state is O
        }else if((input == 'N') && (state == sm.O)){
            // Move state to ON
            state = sm.ON;

         // Input is F and state is O
        }else if((input == 'F') && (state == sm.O)){

            // Move state to F1
            state = sm.F1;

         // input is F and state is F1
        }else if ((input == 'F') && (state == sm.F1)){

            // Move state to OFF
            state = sm.OFF;

        }else{
            // Incorrect characters input, move state back to START
            state = sm.START;
        }

        if(state == sm.ON){
            // Turn red LED ON
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

            // Move state to START
            state = sm.START;

        }else if (state == sm.OFF){
            // Turn red LED OFF
            GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);

            // Move state to START
            state = sm.START;
        }


        UART2_write(uart, &input, 1, &bytesWritten);
    }
}

