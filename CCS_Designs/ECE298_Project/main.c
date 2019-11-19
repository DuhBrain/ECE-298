#include "main.h"
#include "driverlib/driverlib.h"
#include "hal_LCD.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <unistd.h>
/*
 * This project contains some code samples that may be useful.
 *
 */

char ADCState = 0; //Busy state of the ADC
int16_t ADCResult = 0; //Storage for the ADC conversion result
int daytime = 0; //set based on light sensor input
int display_zone = 0;
int params [4] =  {1000,1000,200,200}; // vent1, vent2, irr1, irr2
float temp[2]; //zone1, zone2
int moisture[2]; //zone1, zone 2
int debug =0;
uint8_t uartRxData;


void main(void)
{
    char buttonState = 0; //Current button press state (to allow edge detection)
    int toggle =1;
    int count=0;
    int sw = 0;
    char daytime_msg[20];
    char temp_msg[20];
    char moist_msg[20];
    char curr_zone[20];

    /*
     * Functions with two underscores in front are called compiler intrinsics.
     * They are documented in the compiler user guide, not the IDE or MCU guides.
     * They are a shortcut to insert some assembly code that is not really
     * expressible in plain C/C++. Google "MSP430 Optimizing C/C++ Compiler
     * v18.12.0.LTS" and search for the word "intrinsic" if you want to know
     * more.
     * */

    //Turn off interrupts during initialization
    __disable_interrupt();

    //Stop watchdog timer unless you plan on using it
    WDT_A_hold(WDT_A_BASE);

    // Initializations - see functions for more detail
    Init_GPIO();    //Sets all pins to output low as a default
    Init_PWM();     //Sets up a PWM output
    Init_ADC(GPIO_PORT_P8,GPIO_PIN1,ADC_INPUT_A9);     //Sets up the ADC to sample
    Init_Clock();   //Sets up the necessary system clocks
    Init_UART();    //Sets up an echo over a COM port
    Init_LCD();     //Sets up the LaunchPad LCD display
    Init_PB();

     /*
     * The MSP430 MCUs have a variety of low power modes. They can be almost
     * completely off and turn back on only when an interrupt occurs. You can
     * look up the power modes in the Family User Guide under the Power Management
     * Module (PMM) section. You can see the available API calls in the DriverLib
     * user guide, or see "pmm.h" in the driverlib directory. Unless you
     * purposefully want to play with the power modes, just leave this command in.
     */
    PMM_unlockLPM5(); //Disable the GPIO power-on default high-impedance mode to activate previously configured port settings

    //All done initializations - turn interrupts back on.
    __enable_interrupt();
//    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); //LED3 blue for moisture
//    GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0); //LED4 blue for moisture
//    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); //LED1 green for temp
//    GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3);

    //set mux to be disabled initially so that nothing is connected to pwm
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);

    setup_parameters();

    while(1) //Do this when you want an infinite loop of code
    {
        //Buttons SW1 and SW2 are active low (1 until pressed, then 0)  //  >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>to be changed
       /* if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 1) ) //Look for rising edge
        {
            Timer_A_stop(TIMER_A0_BASE);    //Shut off PWM signal

            sw=1; //reset switch

        }
        if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN) == 0) & sw==1) //Look for falling edge
        {
            if(toggle==0 ){
                param.dutyCycle = 2000;
                toggle=1;


            }else if (toggle ==1 ){
                param.dutyCycle = 1000;
                toggle=0;
            }
            sw =0;//deactivate switching

            Timer_A_outputPWM(TIMER_A0_BASE, &param);   //Turn on PWM
        }*/  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<to be changed


        //J3: A3 is temp A4 is moisture sensor
        get_sensor_info(0,GPIO_PORT_P1, GPIO_PIN3, ADC_INPUT_A3, GPIO_PORT_P1,GPIO_PIN4, ADC_INPUT_A4); //get conditions for zone 0
        //J4 A6 is temp, a5 is moisture
        get_sensor_info(1, GPIO_PORT_P1, GPIO_PIN6, ADC_INPUT_A6, GPIO_PORT_P1,GPIO_PIN5, ADC_INPUT_A5); //get conditions for zone 1


        //now check environmental conditions and run motors + LEDs if needed
        check_conditions(0);
        check_conditions(1);

        sprintf(curr_zone,"ZONE %d", display_zone);
        displayScrollText(curr_zone);

        sprintf(daytime_msg,"DAY %d", daytime);
        displayScrollText(daytime_msg);

        sprintf(temp_msg," TEMP %d",(int)temp[display_zone]);
        displayScrollText(temp_msg);

        sprintf(moist_msg," MOIST %d",moisture[display_zone]);
        displayScrollText(moist_msg);

        // Select M0
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2); //i0
        GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3); //i1

        if(display_zone){ //for testing motors
            param.dutyCycle = 2000;
            Timer_A_outputPWM(TIMER_A0_BASE, &param);
        }else{

            param.dutyCycle = 1000;
            Timer_A_outputPWM(TIMER_A0_BASE, &param);
        }


//        showHex(daytime);

    }

    /*
     * You can use the following code if you plan on only using interrupts
     * to handle all your system events since you don't need any infinite loop of code.
     *
     * //Enter LPM0 - interrupts only
     * __bis_SR_register(LPM0_bits);
     * //For debugger to let it know that you meant for there to be no more code
     * __no_operation();
    */

}

void Init_GPIO(void)
{
    // Set all GPIO pins to output low to prevent floating input and reduce power consumption
    GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P6, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3|GPIO_PIN4|GPIO_PIN5|GPIO_PIN6|GPIO_PIN7);

    //Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);
    //Set LED1 and LED2 as outputs
//    GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);
    //blue green leds
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN2);
    GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3);
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);
    //mux
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);//mux_en
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);// i0
    GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);//i1

    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);//enable is active low
}

/* Clock System Initialization */
void Init_Clock(void)
{
    /*
     * The MSP430 has a number of different on-chip clocks. You can read about it in
     * the section of the Family User Guide regarding the Clock System ('cs.h' in the
     * driverlib).
     */

    /*
     * On the LaunchPad, there is a 32.768 kHz crystal oscillator used as a
     * Real Time Clock (RTC). It is a quartz crystal connected to a circuit that
     * resonates it. Since the frequency is a power of two, you can use the signal
     * to drive a counter, and you know that the bits represent binary fractions
     * of one second. You can then have the RTC module throw an interrupt based
     * on a 'real time'. E.g., you could have your system sleep until every
     * 100 ms when it wakes up and checks the status of a sensor. Or, you could
     * sample the ADC once per second.
     */
    //Set P4.1 and P4.2 as Primary Module Function Input, XT_LF
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN1 + GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set external clock frequency to 32.768 KHz
    CS_setExternalClockSource(32768);
    // Set ACLK = XT1
    CS_initClockSignal(CS_ACLK, CS_XT1CLK_SELECT, CS_CLOCK_DIVIDER_1);
    // Initializes the XT1 crystal oscillator
    CS_turnOnXT1LF(CS_XT1_DRIVE_1);
    // Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_SMCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
    // Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(CS_MCLK, CS_DCOCLKDIV_SELECT, CS_CLOCK_DIVIDER_1);
}

/* UART Initialization */
void Init_UART(void)
{
    /* UART: It configures P1.0 and P1.1 to be connected internally to the
     * eSCSI module, which is a serial communications module, and places it
     * in UART mode. This let's you communicate with the PC via a software
     * COM port over the USB cable. You can use a console program, like PuTTY,
     * to type to your LaunchPad. The code in this sample just echos back
     * whatever character was received.
     */

    //Configure UART pins, which maps them to a COM port over the USB cable
    //Set P1.0 and P1.1 as Secondary Module Function Input.
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN0, GPIO_PRIMARY_MODULE_FUNCTION);

    /*
     * UART Configuration Parameter. These are the configuration parameters to
     * make the eUSCI A UART module to operate with a 9600 baud rate. These
     * values were calculated using the online calculator that TI provides at:
     * http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
     */

    //SMCLK = 1MHz, Baudrate = 9600
    //UCBRx = 6, UCBRFx = 8, UCBRSx = 17, UCOS16 = 1
    EUSCI_A_UART_initParam param = {0};
        param.selectClockSource = EUSCI_A_UART_CLOCKSOURCE_SMCLK;
        param.clockPrescalar    = 6;
        param.firstModReg       = 8;
        param.secondModReg      = 17;
        param.parity            = EUSCI_A_UART_NO_PARITY;
        param.msborLsbFirst     = EUSCI_A_UART_LSB_FIRST;
        param.numberofStopBits  = EUSCI_A_UART_ONE_STOP_BIT;
        param.uartMode          = EUSCI_A_UART_MODE;
        param.overSampling      = 1;

    if(STATUS_FAIL == EUSCI_A_UART_init(EUSCI_A0_BASE, &param))
    {
        return;
    }

    EUSCI_A_UART_enable(EUSCI_A0_BASE);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

    // Enable EUSCI_A0 RX interrupt
    EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
#pragma vector=USCI_A0_VECTOR
__interrupt
void EUSCIA0_ISR(void)
{
    uint8_t RxStatus = EUSCI_A_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);

    EUSCI_A_UART_clearInterrupt(EUSCI_A0_BASE, RxStatus);

    if (RxStatus)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, EUSCI_A_UART_receiveData(EUSCI_A0_BASE));
        //uartRxData = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);
    }
}

/* PWM Initialization */
void Init_PWM(void)
{
    /*
     * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
     * flip an output bit every cycle in software. The catch is that it limits which
     * pins you can use to output the signal, whereas manually flipping an output bit
     * means it can be on any GPIO. This function populates a data structure that tells
     * the API to use the timer as a hardware-generated PWM source.
     *
     */
    //Generate PWM - Timer runs in Up-Down mode
    param.clockSource           = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider    = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param.timerPeriod           = TIMER_A_PERIOD; //Defined in main.h
    param.compareRegister       = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param.compareOutputMode     = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle             = HIGH_COUNT; //Defined in main.h

    //PWM_PORT PWM_PIN (defined in main.h) as PWM output
    GPIO_setAsPeripheralModuleFunctionOutputPin(PWM_PORT, PWM_PIN, GPIO_PRIMARY_MODULE_FUNCTION);
}
void Init_PB(){
    P2IE  = 0x00;
    P2IFG  = 0x00;
    P1IE  = 0x00;
    P1IFG  = 0x00;
    GPIO_enableInterrupt(SW1_PORT,SW1_PIN); //enable interrupt on port 2.6
    GPIO_selectInterruptEdge(SW1_PORT,SW1_PIN,0x1); //want to set to high to low transition
    GPIO_enableInterrupt(SW2_PORT,SW2_PIN); //enable interrupt on port 2.6
    GPIO_selectInterruptEdge(SW2_PORT,SW2_PIN,0x1); //want to set to high to low transition


}
#pragma vector=PORT1_VECTOR //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
__interrupt
void PB1_ISR(void){
    display_zone ^= 1;
    GPIO_clearInterrupt(SW1_PORT,SW1_PIN);

}
#pragma vector=PORT2_VECTOR
__interrupt
void PB2_ISR(void){
    debug ^= 1;
    GPIO_clearInterrupt(SW2_PORT,SW2_PIN);

}

void Init_ADC(int port, int pin, int channel)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(port, pin, GPIO_PRIMARY_MODULE_FUNCTION);//port, pin

    //Initialize the ADC Module
    /*
     * Base Address for the ADC Module
     * Use internal ADC bit as sample/hold signal to start conversion
     * USE MODOSC 5MHZ Digital Oscillator as clock source
     * Use default clock divider of 1
     */
    ADC_init(ADC_BASE,
             ADC_SAMPLEHOLDSOURCE_SC,
             ADC_CLOCKSOURCE_ADCOSC,
             ADC_CLOCKDIVIDER_1);

    ADC_enable(ADC_BASE);

    /*
     * Base Address for the ADC Module
     * Sample/hold for 16 clock cycles
     * Do not enable Multiple Sampling
     */
    ADC_setupSamplingTimer(ADC_BASE,
                           ADC_CYCLEHOLD_16_CYCLES,
                           ADC_MULTIPLESAMPLESDISABLE);

    //Configure Memory Buffer
    /*
     * Base Address for the ADC Module
     * Use input ADC_IN_CHANNEL
     * Use positive reference of AVcc
     * Use negative reference of AVss
     */
    ADC_configureMemory(ADC_BASE, // channel
                        channel,
                        ADC_VREFPOS_AVCC,
                        ADC_VREFNEG_AVSS);

    ADC_clearInterrupt(ADC_BASE,
                       ADC_COMPLETED_INTERRUPT);

    //Enable Memory Buffer interrupt
    ADC_enableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);
}

//ADC interrupt service routine
#pragma vector=ADC_VECTOR
__interrupt
void ADC_ISR(void)
{
    uint8_t ADCStatus = ADC_getInterruptStatus(ADC_BASE, ADC_COMPLETED_INTERRUPT_FLAG);

    ADC_clearInterrupt(ADC_BASE, ADCStatus);

    if (ADCStatus)
    {
        ADCState = 0; //Not busy anymore
        ADCResult = ADC_getResults(ADC_BASE);
    }
}

void get_sensor_info(int zone, int temp_port, int temp_pin, int temp_adc_input, int moist_port, int moist_pin, int moist_adc_input){
    /* this function will check the sensors of the appropriate zone, then set the appropriate variable values*/
    float new_temp [2];
    int tolerance = 3;
    float average_temp;

    //get light sensor data and set daytime varible

    Init_ADC(GPIO_PORT_P8,GPIO_PIN1,ADC_INPUT_A9);

    //where do the following two lines go?
    ADCState = 1;
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
        while(ADCState==1);
    if ((int)ADCResult >10)
        daytime=1;
    else
        daytime=0;



    Init_ADC(temp_port,temp_pin,temp_adc_input);
    ADCState = 1;

    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
    while(ADCState==1);
    new_temp[0] = ADCResult * 3.3/1024;
    new_temp[0] -= 0.5;
    new_temp[0] *= 100;

    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
    while(ADCState==1);
    new_temp[1] = ADCResult * 3.3/1024;
    new_temp[1] -= 0.5;
    new_temp[1] *= 100;


    average_temp = (new_temp[0] + new_temp[1])/2;
    if( abs(new_temp[0] - new_temp[1])< tolerance && average_temp <30 && average_temp >10 ){
        temp[zone] = average_temp;
    }
//    temp[zone] =ADCResult * 3.3/1024;
//    temp[zone] -= 0.5;
//    temp[zone] *=100;





    Init_ADC(moist_port,moist_pin,moist_adc_input);
    ADCState = 1;
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);
    while(ADCState==1);
    moisture[zone] = ADCResult;

}

void check_conditions(int zone){

    /*
     * Motor Definitions:
     *
     * M0 (i1 = LOW, i0 = LOW)      - Ventilation zone 0
     * M1 (i1 = LOW, i0 = HIGH)     - Irrigation zone 0
     * M2 (i1 = HIGH, i0 = LOW)     - Ventilation zone 1
     * M3 (i1 = HIGH, i0 = HIGH)    - Irrigation zone 1
     */

    //check temp
    if(temp[zone]>params[zone]){

        //set led output and motor selectors
        if(zone==0){
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN2); //LED1 green for temp

            // Select M0
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2); //i0
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3); //i1

        }else{
            GPIO_setOutputHighOnPin(GPIO_PORT_P5, GPIO_PIN3); //LED2 green for temp

            // Select M2
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2); //i0
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3); //i1

        }

        // enable
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

        //send out pwm
        param.dutyCycle = 2000;
        Timer_A_outputPWM(TIMER_A0_BASE, &param);
//        sleep(1);

    }else{
        //turn off LEDs
        if(zone==0){
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN2); //LED1 green for temp
        }else{
            GPIO_setOutputLowOnPin(GPIO_PORT_P5, GPIO_PIN3); //LED2 green for temp
        }

        //disable mux
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);

        //rotate motor back
        param.dutyCycle = 1000;
        Timer_A_outputPWM(TIMER_A0_BASE, &param);
//        sleep(1);
    }



    //check moisture
    if(moisture[zone]< params[2+zone]){
        //turn on LED and run motor
        if(zone==0){
            GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7); //LED3 blue for moisture

            // Select M1
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2); //i0
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3); //i1

        }else{
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0); //LED4 blue for moisture

            // Select M3
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2); //i0
            GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3); //i1

        }

        // enable
        GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);

        //send out pwm
        param.dutyCycle = 2000;
        Timer_A_outputPWM(TIMER_A0_BASE, &param);
//        sleep(1);

    }else{
        //turn off LED and rotate motor other direction
        if(zone==0){
            GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7); //LED3 blue for moisture
        }else{
            GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0); //LED4 blue for moisture
        }

        //disable mux
        GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);

        //rotate motor back
        param.dutyCycle = 1000;
        Timer_A_outputPWM(TIMER_A0_BASE, &param);
//        sleep(1);
    }

}

void setup_parameters( void )
{
    char message[] = "Please input desired temperature threshold (2 digits) and press Enter\n";

    int i;
    for ( i = 0 ; i < sizeof(message) ; i++ )
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, (int) message[i]);
    }
}


