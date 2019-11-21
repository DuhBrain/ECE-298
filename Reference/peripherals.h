#include "main.h"
#include "hal_LCD.h"
#include <stdbool.h>
#include <stdio.h>

void welcomeMsgLCD(void); /* welcome message on LCD*/

void welcomeMsgCLI(void); /* welcome message on CLI */

_Bool checkLight(_Bool light); /* check light sensor */

void displayLCD(_Bool zone, unsigned char *temp, unsigned char *soil); /* display on LCD */

void uartDisplay(uint8_t *sendText, uint8_t length); /* send to UART */

void Motor_Control(_Bool ON, int Motor); /* activate motors */
#define MOTOR_CYCLES 20
