#include "peripherals.h"


/* Check light status */
_Bool checkLight (_Bool light)
{
    if ((GPIO_getInputPinValue(ldr_Port, ldr_Pin) == 1) && (! light)) /* check night -> day */
    {
        uint8_t txMsg[50]; /* CLI message */
        strcpy((char*) txMsg, "Day Mode");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("DAY MODE");
        turnOffSoilMotors();
        return true;
    }

    else if ((GPIO_getInputPinValue(ldr_Port, ldr_Pin) == 0) && (light)) /* check day -> night */
    {
        uint8_t txMsg[50]; /* CLI message */
        strcpy((char*) txMsg, "Night mode");
        uartDisplay(txMsg, strlen((char*) txMsg));
        displayScrollText("NIGHT MODE");
        turnOffTempMotors();
        return false;
    }
    return light; /* no light change */
}


/* LCD display */
void displayLCD(_Bool zone, unsigned char *temp, unsigned char *soil)
{
    showChar('T', pos1);
    showChar('M', pos4);

    if (! zone) /* zone 1*/
    {
        showChar(((temp[0]/10)+'0'), pos2);
        showChar(((temp[0]%10)+'0'), pos3);
        showChar(((soil[0]/10)+'0'), pos5);
        showChar(((soil[0]%10)+'0'), pos6);
    }
    else /* zone 2*/
    {
        showChar(((temp[1]/10)+'0'), pos2);
        showChar(((temp[1]%10)+'0'), pos3);
        showChar(((soil[1]/10)+'0'), pos5);
        showChar(((soil[1]%10)+'0'), pos6);
    }
}


/* Tx to UART */
void uartDisplay(uint8_t *sendText, uint8_t length)
{
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
    while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */

    int i;
    for (i = 0 ; i < length ; i++)
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, sendText[i]); /* send message */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
    }

    if ((sendText[0] != '>') && (sendText[0] != '#') && (sendText[0] != ' ')) /* if not enter key or welcome message, it was command, make new line */
    {
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 13); /* send carrier return*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 10); /* send new line*/
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, '>'); /* send new prompt */
        while(EUSCI_A_UART_queryStatusFlags(EUSCI_A0_BASE, EUSCI_A_UART_BUSY)) {} /* wait for UART to be free - stop busy */
        EUSCI_A_UART_transmitData(EUSCI_A0_BASE, 32); /* send space*/
    }
}


/* Activate motors */
void Motor_Control(_Bool ON, int Motor)
{
    int i;
    uint8_t Port;
    uint16_t Pin;

    switch(Motor)
    {
    case 0: //Motor T1
        Port = GPIO_PORT_P1;
        Pin = GPIO_PIN7;
        break;

    case 1: //Motor S1
        Port = GPIO_PORT_P1;
        Pin = GPIO_PIN6;
        break;

    case 2: //Motor T2
        Port = GPIO_PORT_P2;
        Pin = GPIO_PIN7;
        break;

    case 3: //Motor S2
        Port = GPIO_PORT_P5;
        Pin = GPIO_PIN1;
        break;

    default:
        displayScrollText("INCORRECT MOTOR INPUT");
    }

    if (ON) /* motor on */
    {
        for(i = 0 ; i < MOTOR_CYCLES ; i++)
        {
            GPIO_setOutputHighOnPin(Port, Pin);
            __delay_cycles(2000);
            GPIO_setOutputLowOnPin(Port, Pin);
            __delay_cycles(18000);
        }
    }

    else /* motor off */
    {
        for(i = 0 ; i < MOTOR_CYCLES ; i++)
        {
            GPIO_setOutputHighOnPin(Port, Pin);
            __delay_cycles(1000);
            GPIO_setOutputLowOnPin(Port, Pin);
            __delay_cycles(19000);
        }
    }
}


/* LCD welcome message */
void welcomeMsgLCD(void)
{
    clearLCD();
    __delay_cycles(500000);

    showChar('G', pos1);
    showChar('R', pos2);
    showChar('O', pos3);
    showChar('U', pos4);
    showChar('P', pos5);
    showChar('2', pos6);
    __delay_cycles(2000000);
    clearLCD();
    __delay_cycles(500000);

    showChar('G', pos1);
    showChar('R', pos2);
    showChar('E', pos3);
    showChar('E', pos4);
    showChar('N', pos5);
    __delay_cycles(1000000);

    showChar('H', pos1);
    showChar('O', pos2);
    showChar('U', pos3);
    showChar('S', pos4);
    showChar('E', pos5);
    __delay_cycles(1000000);
    clearLCD();
    __delay_cycles(500000);
}


/* CLI welcome message */
void welcomeMsgCLI(void)
{
    uint8_t cliWelcome[100];
    int cliIndex;
    for (cliIndex = 0 ; cliIndex < 100 ; cliIndex++)
        cliWelcome[cliIndex]= 0; /* initialize welcome message */

    strcpy((char*) cliWelcome, "#               ECE 298 - Group 2              #");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, "#         Lehel Balla & Artiom Tsimkin         #");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, "#   Greenhouse Agriculture Monitoring System   #\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " Zones 1 and 2 have and a temperature sensor and a moisture sensor each.");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " Zone and light changes will be displayed below.\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " Use the CLI commands to ENABLE and DISABLE the motors in each zone:");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " motor t1 on     motor s1 on     motor t2 on     motor s2 on");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " motor t1 off    motor s1 off    motor t2 off    motor s2 off\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " Threshold for each motor (0-100) can be set independently with the SET command:");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " set t1 <num>    set s1 <num>    set t2 <num>    set s2 <num>\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " Use command \"display\" to show sensor and actuator info.");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, " At any time use command \"?\" to see this menu again.\r\n");
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
    strcpy((char*) cliWelcome, "> "); /* prompt */
    uartDisplay(cliWelcome, strlen((char*) cliWelcome));
}
