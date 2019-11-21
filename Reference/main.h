#ifndef MAIN_H_
#define MAIN_H_

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "driverlib/driverlib.h"
#include "peripherals.h"
#include "hal_LCD.h"

#define TIMER_A_PERIOD  20000 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      1500  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

//Output pin to buzzer
#define PWM_PORT        GPIO_PORT_P1
#define PWM_PIN         GPIO_PIN7
//LaunchPad LED1 - note unavailable if UART is used
#define LED1_PORT       GPIO_PORT_P1
#define LED1_PIN        GPIO_PIN0
//LaunchPad LED2
#define LED2_PORT       GPIO_PORT_P4
#define LED2_PIN        GPIO_PIN0
//LaunchPad Pushbutton Switch 1
#define SW1_PORT        GPIO_PORT_P1
#define SW1_PIN         GPIO_PIN2
//LaunchPad Pushbutton Switch 2
#define SW2_PORT        GPIO_PORT_P2
#define SW2_PIN         GPIO_PIN6
//Input to ADC
#define ADC_PORT        GPIO_PORT_P1
#define ADC_PIN         GPIO_PIN5
#define ADC_CHANNEL     ADC_INPUT_A5

#define ldr_Port GPIO_PORT_P5
#define ldr_Pin GPIO_PIN0

#define motor0_led_PORT GPIO_PORT_P1
#define motor0_led_PIN GPIO_PIN3
#define motor1_led_PORT GPIO_PORT_P1
#define motor1_led_PIN GPIO_PIN4
#define motor2_led_PORT GPIO_PORT_P8
#define motor2_led_PIN GPIO_PIN2
#define motor3_led_PORT GPIO_PORT_P8
#define motor3_led_PIN GPIO_PIN3

#define mux0_PORT GPIO_PORT_P5
#define mux0_PIN GPIO_PIN2
#define mux1_PORT GPIO_PORT_P5
#define mux1_PIN GPIO_PIN3

#define adc_resolution 1024
#define temp1 1
#define soil1 2
#define temp2 3
#define soil2 4
#define zone1 0
#define zone2 1

#define cliBufferSize 15

#define TEMP1_CALIBRATION 50
#define SOIL1_CALIBRATION 160
#define TEMP2_CALIBRATION 50
#define SOIL2_CALIBRATION 168

void Init_GPIO(void);
void Init_Clock(void);
void Init_UART(void);
void Init_PWM(void);
void Init_ADC(void);

void globalInit(void);
void adc_measure(void);
void zone_select(void);
void adc_analyze(void);
void sensor_select(void);
void update_motors(void);
void uartTransmit(void);
void motorInit();
void turnOffTempMotors(void);
void turnOffSoilMotors(void);


Timer_A_outputPWMParam param; //Timer configuration data structure for PWM

#endif /* MAIN_H_ */
