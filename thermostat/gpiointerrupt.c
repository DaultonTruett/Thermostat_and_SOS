/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
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
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
// enables snprintf() - DT
#include <stdio.h>

#include <unistd.h>


/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

// Timer driver
#include <ti/drivers/Timer.h>


// I2C globals
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    {0x48, 0x0000, "11X"},
    {0x49, 0x0000, "116"},
    {0x41, 0x0001, "006"}
};

uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;


// UART2 globals
#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

char output[64];
size_t bytesWritten = 0;
size_t bytesRead;
uint32_t status = UART2_STATUS_SUCCESS;

// I2C handle - global
I2C_Handle i2c;

// UART2 handle - global
UART2_Handle uart;

// Timer handle - global
Timer_Handle timer0;


// general globals
volatile int16_t temperature;
volatile int16_t setpoint;
volatile unsigned char heat;
volatile int16_t seconds;

volatile unsigned char timerFlag;

// periods for each tasks to execute - DT
volatile unsigned long timeSinceReadBtnInput;
volatile unsigned long timeSinceReadTemp;
volatile unsigned long timeSinceLED_Data;

volatile unsigned char taskPeriodGCD;


void initI2C(void){
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver -"));

    // Driver init
    I2C_init();

    // config
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // open driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);

    if(i2c == NULL){
        DISPLAY(snprintf(output, 64, "FAILED\n\r"));
        while(1);
    };

    DISPLAY(snprintf(output, 32, "Passed\n\r"));

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;

    for(i = 0; i < 3; ++i){
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;

        DISPLAY(snprintf(output, 64, "Is this %s: ", sensors[i].id));

        if(I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Yes\n\r"));

            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"));
    }
    if(found){
        DISPLAY(snprintf(output, 64, "Detected TMP %s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress));
    }else{
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"));
    }
}

int16_t readTemp(void){

    i2cTransaction.readCount = 2;

    if(I2C_transfer(i2c, &i2cTransaction)){
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        if(rxBuffer[0] & 0x80){
            temperature |= 0xF000;
        };

    }else{
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status));
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"));
    };

    return temperature;
};


void initUART2(void){
    UART2_Params uartParams;

    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    //open driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if(uart == NULL){
        while(1);
    };
};

// Task Schedule code to execute tasks at their specified times in ms - DT
void taskScheduler(){
    if(timeSinceReadBtnInput == 2){
         if(temperature >= setpoint){
             heat = 0;
         }else{
             heat = 1;
         };

         timeSinceReadBtnInput = 0;
     };

     if(timeSinceReadTemp == 5){
         temperature = readTemp();

         timeSinceReadTemp = 0;
     };

     if(timeSinceLED_Data == 10){
         if(!heat){
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
         }else{
             GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
         };

         seconds += 1;
         DISPLAY(snprintf(output, 64, "<Temp: %02d, Set point: %02d, Heat: %d, Time: %04d>\n\r", temperature, setpoint, heat, seconds));

         timeSinceLED_Data = 0;
     };
};

// Timer callback Function called every 100ms (GCD for all tasks), will determine which tasks to run dependent on elapsed time - DT
void timerCallback(Timer_Handle myHandle, int_fast16_t status){

    // raise flag - DT
    timerFlag = 1;
};


void initTimer(void){
    // Timer handle - global
    // Timer handle and params for accessing task times - DT
    Timer_Params params;

    Timer_init();
    Timer_Params_init(&params);

    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    timer0 = Timer_open(CONFIG_TIMER_0, &params);

    if(timer0 == NULL){
        while(1){};
    };
    if(Timer_start(timer0) == Timer_STATUS_ERROR){
        while(1){};
    };

    // Start timer - DT
    Timer_start(timer0);
};

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0 && CONFIG_GPIO_BUTTON_1.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index){
    setpoint += 1;
};

void gpioButtonFxn1(uint_least8_t index){
    setpoint -= 1;
};


/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0){

    // Driver initialization - DT
    initUART2();
    initI2C();
    GPIO_init();
    initTimer();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);


    /* Turn on user LED */
    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    // Initialize global variables sent to UART2 - DT
    temperature = readTemp();
    setpoint = temperature;
    heat = 0;
    seconds = 0;


    // initial times for tasks - DT
    timeSinceReadBtnInput = 0;
    timeSinceReadTemp = 0;
    timeSinceLED_Data = 0;
    taskPeriodGCD = 1;

    // initial timerFlag - DT
    timerFlag = 0;

    // Keep the application running - DT
    while(1){

        // wait for timer to call and raise flag, 100ms as this is the GCD amongst all tasks
        // ensures all tasks will execute at their scheduled times - DT
        while(!timerFlag){};

        // lower flag - DT
        timerFlag = 0;

        // Increase time since each task execution by a ratio of the Timer period before calling timeCallback()
        // and checking/executing tasks- DT
        timeSinceReadBtnInput += taskPeriodGCD;
        timeSinceReadTemp += taskPeriodGCD;
        timeSinceLED_Data += taskPeriodGCD;

        // Call the taskScheduler every 100ms - DT
        taskScheduler();
    };

    return (NULL);
};

