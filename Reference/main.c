#include "main.h"

volatile static _Bool light = true;                         /* light state */
volatile static _Bool buttonPress = false;                  /* zone select button state */
static _Bool adcBusy = false; //Busy state of the ADC       /* ADC busy flag */
static _Bool zone = zone1;                                  /* zone 1 or zone 2 */
static _Bool adc_done = false;                              /* ADC done flag */
static _Bool mux0 = true;                                   /* MUX control pin 0 */
static _Bool mux1 = true;                                   /* MUX control pin 1 */
static _Bool uartReceived = false;                          /* UART receive flag */
static _Bool motorState[4] = {false, false, false, false};  /* motor states */
static _Bool motorEnable[4] = {true, true, true, true};     /* motor enables */
static char measurement = temp1;                            /* current measurement */
static uint8_t temp[2] = {0, 0};                            /* temperature values */
static uint8_t soil[2] = {0, 0};                            /* soil values */
static uint8_t cliBuffer[cliBufferSize];                    /* CLI output buffer */
static uint8_t cliIndex = 0;                                /* CLI buffer index */
static int16_t adcResult = 0;                               /* ADC result */
static uint8_t temp_threshold[2] = {25, 25};               /* temperature motors threshold */
static uint8_t soil_threshold[2] = {25, 25};               /* soil motors thresholds */


void main(void)
{
    globalInit();                               /* global CPU init */
    motorInit();                                /* motor init */
    welcomeMsgLCD();                            /*LCD welcome message */
    welcomeMsgCLI();                            /* CLI welcome message */

    int iInit;
    for (iInit = 0 ; iInit < cliBufferSize ; iInit++) /* initialize receive buffer */
        cliBuffer[iInit] = 0;

    while(1)
    {
        light = checkLight(light);              /* check day or night */

        if (! adcBusy)                          /* ADC conversion if not busy */
        {
            sensor_select();                    /* set mux to choose sensor */
            adc_measure();                      /* start ADC */
        }

        if (adc_done)                           /* ADC done */
        {
            adc_done = false;
            adc_analyze();                      /* get ADC results */
        }

        zone_select();                          /* check zone selection */
        displayLCD(zone, temp, soil);           /* display results */

        update_motors();                        /* temp motor */

        if (uartReceived)                       /* UART communication */
            uartTransmit();

        __delay_cycles(100000);                 /* chill */
    }
}

void Init_GPIO(void)
{
    /* Set all GPIO pins to output low to prevent floating input and reduce power consumption */
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

    /* Set LaunchPad switches as inputs - they are active low, meaning '1' until pressed */
    GPIO_setAsInputPinWithPullUpResistor(SW1_PORT, SW1_PIN);
    GPIO_setAsInputPinWithPullUpResistor(SW2_PORT, SW2_PIN);

    /* Set LED1 and LED2 as outputs */
    /* GPIO_setAsOutputPin(LED1_PORT, LED1_PIN); //Comment if using UART */
    GPIO_setAsOutputPin(LED2_PORT, LED2_PIN);

    /* peripherals */
    GPIO_setAsInputPinWithPullDownResistor(ldr_Port, ldr_Pin);
}

/* Clock System Initialization */
void Init_Clock(void)
{
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
    uint8_t received_data = EUSCI_A_UART_receiveData(EUSCI_A0_BASE);

    if ((RxStatus) && !((received_data == 0x7F) && (cliIndex == 0))) /* received correct package, and not a backspace in the begining */
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, received_data); /* echo */
    }

   if (received_data == '\r') /* enter key */
        uartReceived = true;

   else if (received_data == 0x7F) /* backspace key */
   {
       if (cliIndex > 0) /* if buffer not empty */
       {
           cliIndex--;
           if (cliIndex < cliBufferSize) /* within buffer range */
               cliBuffer[cliIndex] = 0; /* remove last char from buffer */
       }
   }

   else if (cliIndex < cliBufferSize) /* other keys */
   {
       if ((isalpha(tolower(received_data))) || (isdigit(received_data)) || (received_data == ' ') || (received_data == '?')) /* legal keys */
           cliBuffer[cliIndex] = tolower(received_data); /* store key */
       cliIndex++;
   }
   else
       cliIndex++;
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


void Init_ADC(void)
{
    /*
     * To use the ADC, you need to tell a physical pin to be an analog input instead
     * of a GPIO, then you need to tell the ADC to use that analog input. Defined
     * these in main.h for A9 on P8.1.
     */

    //Set ADC_IN to input direction
    GPIO_setAsPeripheralModuleFunctionInputPin(ADC_PORT, ADC_PIN, GPIO_PRIMARY_MODULE_FUNCTION);

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
    ADC_configureMemory(ADC_BASE,
                        ADC_CHANNEL,
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
        adcBusy = false; /* Not busy anymore */
        adcResult = ADC_getResults(ADC_BASE); /* save result */
        adc_done = true; /* conversion complete */
    }
}


/* start ADC */
void adc_measure (void)
{
    adcBusy = true; /* ADC is busy */
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL); /* start */
}


/* check ADC result */
void adc_analyze(void)
{
    uint8_t result;

    if (measurement == temp1) /* temperature 1 measurement */
    {
        result = TEMP1_CALIBRATION*(((float)adcResult/adc_resolution));
        if (result > 99)
            result = 99;
        temp[0] = result;
    }
    else if (measurement == soil1) /* soil 1 measurement */
    {
        result = SOIL1_CALIBRATION*(((float)adcResult/adc_resolution));
        if (result > 99)
            result = 99;
        soil[0] = result;
    }
    else if (measurement == temp2) /* temperature 2 measurement */
    {
        result = TEMP2_CALIBRATION*(((float)adcResult/adc_resolution));
        if (result > 99)
            result = 99;
        temp[1] = result;
    }
    else if (measurement == soil2) /* soil 2 measurement */
    {
        result = SOIL2_CALIBRATION*(((float)adcResult/adc_resolution));
        if (result > 99)
            result = 99;
        soil[1] = result;
    }
}


/* check button zone selector */
void zone_select(void)
{
    if ((! GPIO_getInputPinValue(SW1_PORT, SW1_PIN)) && (! buttonPress)) /* button press */
    {
        buttonPress = true;
        uint8_t txMsg[50]; /* CLI message */

        int i;
        for (i = 0 ; i < cliBufferSize ; i++) /* clear receive buffer */
            cliBuffer[i] = 0;

        if (! zone) /* zone 1 -> zone 2*/
        {
            zone = zone2;
            strcpy((char*) txMsg, "Monitoring ZONE 2");
            uartDisplay(txMsg, strlen((char*) txMsg));
            displayScrollText("ZONE 2");
        }
        else /* zone 2 -> zone 1*/
        {
            zone = zone1;
            strcpy((char*) txMsg, "Monitoring ZONE 1");
            uartDisplay(txMsg, strlen((char*) txMsg));
            displayScrollText("ZONE 1");
        }
    }
    else if ((GPIO_getInputPinValue(SW1_PORT, SW1_PIN)) && (buttonPress)) /* botton release */
        buttonPress = false;
}


/* setup MUX controls */
void sensor_select(void)
{
    if ((! mux1) && (! mux0)) /* update mux control: 0->1 */
    {
        mux0 = true;
        mux1 = false;
        GPIO_setOutputHighOnPin(mux0_PORT, mux0_PIN); /* MUX control */
        GPIO_setOutputLowOnPin(mux1_PORT, mux1_PIN); /* MUX control */
        measurement = soil1;
    }

    else if ((! mux1) && (mux0)) /* update mux control: 1->2 */
    {
        mux0 = false;
        mux1 = true;
        GPIO_setOutputHighOnPin(mux1_PORT, mux1_PIN); /* MUX control */
        GPIO_setOutputLowOnPin(mux0_PORT, mux0_PIN); /* MUX control */
        measurement = temp2;
    }

    else if ((mux1) && (! mux0)) /* update mux control: 2->3 */
    {
        mux0 = true;
        mux1 = true;
        GPIO_setOutputHighOnPin(mux1_PORT, mux1_PIN); /* MUX control */
        GPIO_setOutputHighOnPin(mux0_PORT, mux0_PIN); /* MUX control */
        measurement = soil2;
    }

    else if ((mux1) && (mux0)) /* update mux control: 3->0 */
    {
        mux0 = false;
        mux1 = false;
        GPIO_setOutputLowOnPin(mux1_PORT, mux1_PIN); /* MUX control */
        GPIO_setOutputLowOnPin(mux0_PORT, mux0_PIN); /* MUX control */
        measurement = temp1;
    }
}


/* Activate motors */
void update_motors(void)
{
    if (light && motorEnable[0]) /* temp motor 1 */
    {
        if ((temp[0] > temp_threshold[0]) && (! motorState[0])) /* Temperature 1 too high */
        {
            GPIO_setOutputHighOnPin(motor0_led_PORT, motor0_led_PIN);
            Motor_Control(1, 0); /* activate motor */
            motorState[0] = true;
        }
        else if ((temp[0] < temp_threshold[0]) && (motorState[0])) /* Temperature 1 is fine */
        {
            GPIO_setOutputLowOnPin(motor0_led_PORT, motor0_led_PIN);
            Motor_Control(0, 0); /* stop motor */
            motorState[0] = false;
        }
    }

    if (! light && motorEnable[1])  /* soil motor 1 */
    {
        if ((soil[0] < soil_threshold[0]) && (! motorState[1])) /* moisture 1 too low */
        {
            GPIO_setOutputHighOnPin(motor1_led_PORT, motor1_led_PIN);
            Motor_Control(1, 1); /* activate motor */
            motorState[1] = true;
        }
        else if ((soil[0] > soil_threshold[0]) && (motorState[1])) /* moisture 1 is fine */
        {
            GPIO_setOutputLowOnPin(motor1_led_PORT, motor1_led_PIN);
            Motor_Control(0, 1); /* stop motor */
            motorState[1] = false;
        }
    }

    if (light && motorEnable[2]) /* temp motor 2 */
    {
        if ((temp[1] > temp_threshold[1]) && (! motorState[2])) /* Temperature 2 too high */
        {
            GPIO_setOutputHighOnPin(motor2_led_PORT, motor2_led_PIN);
            Motor_Control(1, 2); /* activate motor */
            motorState[2] = true;
        }
        else if ((temp[1] < temp_threshold[1]) && (motorState[2])) /* Temperature 2 is fine */
        {
            GPIO_setOutputLowOnPin(motor2_led_PORT, motor2_led_PIN);
            Motor_Control(0, 2); /* stop motor */
            motorState[2] = false;
        }
    }

    if (! light && motorEnable[3]) /* soil motor 2 */
    {
        if ((soil[1] < soil_threshold[1]) && (! motorState[3])) /* moisture 2 too low */
        {
            GPIO_setOutputHighOnPin(motor3_led_PORT, motor3_led_PIN);
            Motor_Control(1, 3); /* activate motor */
            motorState[3] = true;
        }
        else if ((soil[1] > soil_threshold[1]) && (motorState[3])) /* moisture 2 is fine */
        {
            GPIO_setOutputLowOnPin(motor3_led_PORT, motor3_led_PIN);
            Motor_Control(0, 3); /* stop motor */
            motorState[3] = false;
        }
     }
}


/* UART communication */
void uartTransmit(void)
{
    uartReceived = false;
    uint8_t txMsg[100]; /* UART TX message */

    int i;
    for (i = 0 ; i < 50 ; i++) /* initialize txMsg buffer */
        txMsg[i] = 0;

    if (! strcmp((char*)cliBuffer, "")) /* enter key*/
    {
        strcpy((char*) txMsg, "> "); /* prompt */
        uartDisplay(txMsg, strlen((char*) txMsg));
    }

    else if (! strcmp((char*)cliBuffer, "?")) /* help key*/
    {
        welcomeMsgCLI();
    }

    else if ((cliBuffer[0] == 's') && (cliBuffer[1] == 'e') && (cliBuffer[2] == 't')) /* threshold set */
    {
        int newThreshold = -1;
        if (isdigit(cliBuffer[9])) /* 3 digit value */
            newThreshold = (cliBuffer[7] - 48)*100 + (cliBuffer[8] - 48)*10 + (cliBuffer[9] - 48);
        else if (isdigit(cliBuffer[8])) /* 2 digit value */
            newThreshold = (cliBuffer[7] - 48)*10 + (cliBuffer[8] - 48);
        else if (isdigit(cliBuffer[7])) /* 1 digit value */
            newThreshold = (cliBuffer[7] - 48);

        if ((newThreshold >= 0) && (newThreshold <= 100)) /* threshold is a valid value */
        {
            if ((cliBuffer[4] == 't') && (cliBuffer[5] == '1')) /*temp1 threshold */
            {
                temp_threshold[0] = newThreshold;
                strcpy((char*) txMsg, "The new threshold for temperature motor 1 was set");
                uartDisplay(txMsg, strlen((char*) txMsg));
            }
            else if ((cliBuffer[4] == 's') && (cliBuffer[5] == '1')) /*soil1 threshold */
            {
                soil_threshold[0] = newThreshold;
                strcpy((char*) txMsg, "The new threshold for soil motor 1 was set");
                uartDisplay(txMsg, strlen((char*) txMsg));
            }
            else if ((cliBuffer[4] == 't') && (cliBuffer[5] == '2')) /*temp2 threshold */
            {
                temp_threshold[1] = newThreshold;
                strcpy((char*) txMsg, "The new threshold for temperature motor 2 was set");
                uartDisplay(txMsg, strlen((char*) txMsg));
            }
            else if ((cliBuffer[4] == 's') && (cliBuffer[5] == '2')) /* soil2 threshold */
            {
                soil_threshold[1] = newThreshold;
                strcpy((char*) txMsg, "The new threshold for soil motor 2 was set");
                uartDisplay(txMsg, strlen((char*) txMsg));
            }
            else /* threshold set error*/
            {
                strcpy((char*) txMsg, "ERROR: Invalid threshold!");
                uartDisplay(txMsg, strlen((char*) txMsg));
            }
        }
        else /* threshold set error*/
        {
            strcpy((char*) txMsg, "ERROR: Invalid threshold!");
            uartDisplay(txMsg, strlen((char*) txMsg));
        }
    }

    else if (! strcmp((char*)cliBuffer, "display")) /* print values */
    {
        sprintf((char*) txMsg," Displaying values:");
        uartDisplay(txMsg, strlen((char*) txMsg));
        sprintf((char*) txMsg,"  Zone 1 Temperature:    %d\t(Threshold: %d)\t\tMotor state: %s", temp[0], temp_threshold[0], motorState[0] ? "ON" : "OFF");
        uartDisplay(txMsg, strlen((char*) txMsg));
        sprintf((char*) txMsg,"  Zone 1 Soil Moisture:  %d\t(Threshold: %d)\t\tMotor state: %s", soil[0], soil_threshold[0], motorState[1] ? "ON" : "OFF");
        uartDisplay(txMsg, strlen((char*) txMsg));
        sprintf((char*) txMsg,"  Zone 2 Temperature:    %d\t(Threshold: %d)\t\tMotor state: %s", temp[1], temp_threshold[1], motorState[2] ? "ON" : "OFF");
        uartDisplay(txMsg, strlen((char*) txMsg));
        sprintf((char*) txMsg,"  Zone 2 Soil Moisture:  %d\t(Threshold: %d)\t\tMotor state: %s\r\n> ", soil[1], soil_threshold[1], motorState[3] ? "ON" : "OFF");
        uartDisplay(txMsg, strlen((char*) txMsg));
    }

    else if (! strcmp((char*)cliBuffer, "motor t1 on")) /* enable temperature motor 1 */
    {
        motorEnable[0] = true;
        strcpy((char*) txMsg, "Temperature motor 1 is enabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR T1 ENABLED");
    }

    else if (! strcmp((char*)cliBuffer, "motor t1 off")) /* disable temperature motor 1 */
    {
        motorEnable[0] = false;
        strcpy((char*) txMsg, "Temperature motor 1 is disabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR T1 DISABLED");
    }

    else if (! strcmp((char*)cliBuffer, "motor s1 on")) /* enable soil motor 1 */
    {
        motorEnable[1] = true;
        strcpy((char*) txMsg, "Soil motor 1 is enabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR S1 ENABLED");
    }

    else if (! strcmp((char*)cliBuffer, "motor s1 off")) /* disable soil motor 1 */
    {
        motorEnable[1] = false;
        strcpy((char*) txMsg, "Soil motor 1 is disabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR S1 DISABLED");
    }

    else if (! strcmp((char*)cliBuffer, "motor t2 on")) /* enable temperature motor 2 */
    {
        motorEnable[2] = true;
        strcpy((char*) txMsg, "Temperature motor 2 is enabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR T2 ENABLED");
    }

    else if (! strcmp((char*)cliBuffer, "motor t2 off")) /* disable temperature motor 2 */
    {
        motorEnable[2] = false;
        strcpy((char*) txMsg, "Temperature motor 2 is disabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR T2 DISABLED");
    }

    else if (! strcmp((char*)cliBuffer, "motor s2 on")) /* enable soil motor 2 */
    {
        motorEnable[3] = true;
        strcpy((char*) txMsg, "Soil motor 2 is enabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR S2 ENABLED");
    }
    else if (! strcmp((char*)cliBuffer, "motor s2 off")) /* disable soil motor 2 */
    {
        motorEnable[3] = false;
        strcpy((char*) txMsg, "Soil motor 2 is disabled");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("MOTOR S2 DISABLED");
    }

    else
    {
        strcpy((char*) txMsg, "ERROR: Invalid command!"); /* error */
        uartDisplay(txMsg, strlen((char*) txMsg));
    }

    cliIndex = 0;
    for (i = 0 ; i < cliBufferSize ; i++) /* clear receive buffer */
        cliBuffer[i] = 0;
}


/* close motors at start */
void motorInit()
{
    Motor_Control(0, 0); /* close temp1 motor */
    showChar('E', pos1);
    Motor_Control(0, 1); /* close soil1 motor */
    showChar('C', pos2);
    Motor_Control(0, 2); /* close temp2 motor */
    showChar('E', pos3);
    Motor_Control(0, 3); /* close soil2 motor */
    showChar('2', pos4);
    showChar('9', pos5);
    showChar('8', pos6);
    __delay_cycles(1000000); /* hold welcome message */
}


/* turn off night soil motors */
void turnOffSoilMotors(void)
{
    Motor_Control(0, 1); /* close soil1 motor */
    motorState[1] = false;
    GPIO_setOutputLowOnPin(motor1_led_PORT, motor1_led_PIN);

    Motor_Control(0, 3); /* close soil2 motor */
    motorState[3] = false;
    GPIO_setOutputLowOnPin(motor3_led_PORT, motor3_led_PIN);

}


/* turn off day temperature motors */
void turnOffTempMotors(void)
{
    Motor_Control(0, 0); /* close temp1 motor */
    motorState[0] = false;
    GPIO_setOutputLowOnPin(motor0_led_PORT, motor0_led_PIN);

    Motor_Control(0, 2); /* close temp2 motor */
    motorState[2] = false;
    GPIO_setOutputLowOnPin(motor2_led_PORT, motor2_led_PIN);
}


/* global CPU init */
void globalInit(void)
{
    __disable_interrupt(); /* disable interrupts */

    WDT_A_hold(WDT_A_BASE); /* cancel watchdog timer */

    Init_GPIO();    /* GPIO init */
    Init_ADC();     /* ADC init */
    Init_Clock();   /* clock init */
    Init_UART();    /* UART init */
    Init_LCD();     /* LCD init */

    PMM_unlockLPM5(); /* Disable GPIO power-on default high-impedance mode to activate previously configured port settings */

    __enable_interrupt(); /* enable interrupts */
}
