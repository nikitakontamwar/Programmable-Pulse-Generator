#include<stdio.h>
#include <string.h>
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include <stdbool.h>
#include<stdlib.h>
#include<math.h>
#include<uart0.h>
#define PF3               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define GREEN_LED           PF3

#define PF1               (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define LED           PF1

// PortE masks
#define AIN0_MASK 8
#define AIN1_MASK 4

//#define PA5               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 5*4)))

#define MAX_CHARS 80
//#define MAX_ARGS 80
#define SPECIAL_CHARS (str[i]== 9 || str[i]==' '|| str[i]== 47|| (str[i]>=33 && str[i]<=44)||(str[i]>=58 && str[i]<=64) ||(str[i]>=91 && str[i]<=96) ||(str[i]>=123 && str[i]<=127))
#define ALPHA_NUM ((str[i+1]==45)||(str[i+1]==46)||(str[i+1]>=48 && str[i+1]<=57)||(str[i+1]>=65 && str[i+1]<=90)||(str[i+1]>=97 && str[i+1]<=122))

#define CLK               (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 0*4)))
#define FSS               (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 1*4)))
#define TX               (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4)))
#define LDAC               (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4)))

#define LDAC_MASK  64      //PA6
#define FSS_MASK 2      //PD1-CS
#define CLK_MASK 1      //PD0-SCK
#define TX_MASK 8    //PD3 -SDI

char str[MAX_CHARS + 1];
char ans[MAX_CHARS][MAX_CHARS];

#define delay4Cycles() __asm(" NOP\n NOP\n NOP\n NOP")

uint32_t phi_1, del_phi_1;
uint32_t phi_2, del_phi_2;
uint16_t LUT[2][4096];
int mode = 0;
int star = 0;
int flag = 0;
float f;
float val = 1;

float freq_A = 0, freq_B = 0;
//char new_str[MAX_ARGS][MAX_CHARS];
uint8_t pos[MAX_CHARS]; //position of each word that is the starting index of each word
uint8_t argCount;   //total number of words


float amp_1;
float ofs_1;

int p;

void initHw()
{
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN
            | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF
            | SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE;

    // Enable clocks for ADC
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R0;
    SYSCTL_RCGCADC_R |= SYSCTL_RCGCADC_R1;
    //SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOE|SYSCTL_RCGC2_GPIOD;

    //configure pins for green LED
    GPIO_PORTF_DIR_R |= 0x08;              // Enable PF1 as Output for green Led
    GPIO_PORTF_DEN_R |= 0x08;                    // Enable Digital for green LED
    GPIO_PORTF_DR2R_R |= 0x08;
    /*
     //configure pins for  LED
     GPIO_PORTF_DIR_R |= 0x02;              // Enable PF1 as Output for green Led
     GPIO_PORTF_DEN_R |= 0x02;                    // Enable Digital for green LED
     GPIO_PORTF_DR2R_R |= 0x02;
     */
    //Enable timer clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;

    // Configure AIN0 and AIN1 as analog outputs

    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE3_AIN0;
    GPIO_PORTE_PCTL_R |= GPIO_PCTL_PE2_AIN1;

    GPIO_PORTE_AFSEL_R |= AIN0_MASK;       // select alternative functions (PE3)
    GPIO_PORTE_AFSEL_R |= AIN1_MASK;       // select alternative functions (PE2)

    GPIO_PORTE_DEN_R &= ~AIN0_MASK;     // turn off digital operation on pin PE3
    GPIO_PORTE_DEN_R &= ~AIN1_MASK;     // turn off digital operation on pin PE2

    GPIO_PORTE_AMSEL_R |= AIN0_MASK;      // turn on analog operation on pin PE3
    GPIO_PORTE_AMSEL_R |= AIN1_MASK;      // turn on analog operation on pin PE2

    /*// Configure ADC for AIN0
     ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
     ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
     ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
     ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
     ADC0_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
     ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

     // Configure ADC for AIN1
     ADC1_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
     ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
     ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
     ADC1_SSMUX3_R = 1;                               // set first sample to AIN1
     ADC1_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
     ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
     */
    // Configure UART0 pins
    GPIO_PORTA_DIR_R |= 2; // enable output on UART0 TX pin: default, added for clarity
    GPIO_PORTA_DEN_R |= 3; // enable digital on UART0 pins: default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // use peripheral to drive PA0, PA1: default, added for clarity
    GPIO_PORTA_PCTL_R &= 0xFFFFFF00;       // set fields for PA0 and PA1 to zero
    GPIO_PORTA_PCTL_R |= GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;
    // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART0 to 115200 baud, 8N1 format
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other UARTs in same status
    delay4Cycles();
    // wait 4 clock cycles
    UART0_CTL_R = 0;                 // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK;                 // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // Enable clocks
    SYSCTL_RCGCSSI_R |= SYSCTL_RCGCSSI_R1;  //SSI3 Module

    // Configure LDAC
    GPIO_PORTA_DIR_R |= LDAC_MASK;                       // make bit 2 an output
    GPIO_PORTA_DR2R_R |= LDAC_MASK;                 // set drive strength to 2mA
    GPIO_PORTA_DEN_R |= LDAC_MASK;                   // enable bit 1 for digital

    // Configure SSI3 pins for SPI configuration
    GPIO_PORTD_DIR_R |= TX_MASK | FSS_MASK | CLK_MASK; // make SSI1 TX, FSS, and CLK outputs
    GPIO_PORTD_DR2R_R |= TX_MASK | FSS_MASK | CLK_MASK; // set drive strength to 2mA
    GPIO_PORTD_AFSEL_R |= TX_MASK | FSS_MASK | CLK_MASK; // select alternative functions
    GPIO_PORTD_PCTL_R = GPIO_PCTL_PD3_SSI1TX | GPIO_PCTL_PD1_SSI1FSS
            | GPIO_PCTL_PD0_SSI1CLK; // map alt fns to SSI3
    GPIO_PORTD_DEN_R |= TX_MASK | FSS_MASK | CLK_MASK; // enable digital operation
    GPIO_PORTD_PUR_R |= CLK_MASK;  // SCLK must be enabled when SPO=1 (see 15.4)

    // Configure the SSI1 as a SPI master, mode 3,12 bit operation, 1 MHz bit rate
    SSI1_CR1_R &= ~SSI_CR1_SSE;       // turn off SSI3 to allow re-configuration
    SSI1_CR1_R = 0;                                    // select master mode
    SSI1_CC_R = 0;                    // select system clock as the clock source
    SSI1_CPSR_R = 10;                  // set bit rate to 1 MHz (if SR=0 in CR0)
    SSI1_CR0_R |= SSI_CR0_FRF_MOTO | SSI_CR0_DSS_16; // set SR=0, mode 3 (SPH=1, SPO=1), 16-bit
    SSI1_CR1_R |= SSI_CR1_SSE;                         // turn on SSI1

    LDAC = 1;
    FSS = 1;

    TIMER1_CTL_R &= ~TIMER_CTL_TAEN;      // turn-off timer before reconfiguring
    TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;    // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD; // configure for periodic mode (count down)
    TIMER1_TAILR_R = 400;      // set load value to 40e6 for 1 Hz interrupt rate
}

void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");
    // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");
    // 6
    __asm("             CBZ  R1, WMS_DONE1");
    // 5+1*3
    __asm("             NOP");
    // 5
    __asm("             NOP");
    // 5
    __asm("             B    WMS_LOOP1");
    // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");
    // 1
    __asm("             CBZ  R0, WMS_DONE0");
    // 1
    __asm("             NOP");
    // 1
    __asm("             B    WMS_LOOP0");
    // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");
    // ---
    // 40 clocks/us + error
}

//Step 1
// Blink led for 500 milliseconds
void flash_led(void)        //step 1
{
    GREEN_LED = 1;
    waitMicrosecond(500000);  //500 ms delay
    GREEN_LED = 0;
}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart0(char c)
{
    while (UART0_FR_R & UART_FR_TXFF)
        ;               // wait if uart0 tx fifo full
    UART0_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart0(char* str)
{
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart0()
{
    while (UART0_FR_R & UART_FR_RXFE)
        ;               // wait if uart0 rx fifo empty
    return UART0_DR_R & 0xFF;                        // get character from fifo
}

int16_t readAdc0Ss3()                               //ch 1
{
    // Configure ADC for AIN0
    ADC0_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
    ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
    ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC0_SSMUX3_R = 0;                               // set first sample to AIN0
    ADC0_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
    ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                // enable SS3 for operation
    ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC0_ACTSS_R & ADC_ACTSS_BUSY)
        ;           // wait until SS3 is not busy
    return ADC0_SSFIFO3_R;                    // get single result from the FIFO
}

int16_t readAdc1Ss3()                                  //ch 2
{
    ADC1_CC_R = ADC_CC_CS_SYSPLL; // select PLL as the time base (not needed, since default value)
    ADC1_ACTSS_R &= ~ADC_ACTSS_ASEN3; // disable sample sequencer 3 (SS3) for programming
    ADC1_EMUX_R = ADC_EMUX_EM3_PROCESSOR; // select SS3 bit in ADCPSSI as trigger
    ADC1_SSMUX3_R = 1;                               // set first sample to AIN1
    ADC1_SSCTL3_R = ADC_SSCTL3_END0;             // mark first sample as the end
    ADC1_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation

    ADC1_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
    while (ADC1_ACTSS_R & ADC_ACTSS_BUSY)
        ;           // wait until SS3 is not busy
    return ADC1_SSFIFO3_R;
}

void getString(void)        //step 2
{
    uint8_t count = 0;
    char c;
    while (1)
    {
        A: c = getcUart0();
        putcUart0(c);
        if ((c == 8) || (c == 127))
        {
            if (count > 0)
            {
                count--;
                goto A;
            }
            else
            {
                goto A;
            }
        }
        else
        {
            if ((c == 10) || (c == 13))
            {
                B: str[count] = 0;
                break;
            }
            else
            {
                if (c >= 32)
                {
                    str[count++] = c;
                    if (count == MAX_CHARS)
                        goto B;
                    else
                        goto A;

                }

                else
                    goto A;

            }
        }
    }

}

void parseString()
{

    uint8_t i, l, j = 0, k = 0, p = 0;
    l = strlen(str);
    if ((str[0] >= 48 && str[0] <= 57) || (str[0] >= 65 && str[0] <= 90)
            || (str[0] >= 97 && str[0] <= 122))
    {
        pos[0] = 0;
        p++;
    }
    for (i = 0; i <= l; i++)
    {
        if ((str[i] == ' ') || SPECIAL_CHARS)
        {
            if (ALPHA_NUM)
            {
                pos[p] = i + 1;
                p++;
                k++;
                j = 0;
            }
            str[i] = '\0';
        }
        else
        {
            ans[k][j] = str[i];
            j++;
        }
    }

    argCount = k + 1;

}

char* getArgString(uint8_t argNo)
{
    if (argNo < argCount)
        return &str[pos[argNo]];

    else
        return "invalid argument number";
}

uint32_t getArgInt(uint8_t argNo)
{
    return atoi(getArgString(argNo));

}

float getArgFloat(uint8_t argNo)
{
    return atof(getArgString(argNo));

}

bool isCommand(char* cmd, uint8_t argNo)
{
    uint16_t cmp;

    cmp = strcmp(&str[pos[0]], cmd);
    if (argNo == (argCount - 1) && cmp == 0)
    {
        putsUart0("valid command");
        putsUart0("\r\n");

        return true;
    }
    else
    {

        // putsUart0("Invalid Command");
        // putsUart0("\r\n");
        return false;
    }
}

void getVoltage()
{

    float v0, v1;
    char c0[50];
    char c1[50];
    int channel = getArgInt(1);
    if (channel == 1)
    {
        v0 = readAdc0Ss3();
        float v0_raw = (v0 * 3.3 / 4096);
        putsUart0("\r\n");
        sprintf(c0, "voltage at AIN0 = %f", v0_raw);
        //sprintf(c0, "voltage at AIN0 = %f", v0);
        putsUart0(c0);
    }
    if (channel == 2)
    {
        v1 = readAdc1Ss3();
        float v1_raw = (v1 * 3.3 / 4096);
        putsUart0("\r\n");
        sprintf(c1, "voltage at AIN1 = %f", v1_raw);
        putsUart0(c1);
    }

}

/*

float readVoltage(int channel)
{

    float v0, v1;
    char c0[50];
    char c1[50];

    if (channel == 1)
    {
        v0 = readAdc0Ss3();
        float v0_raw = (v0 * 3.3 / 4096);
        putsUart0("\r\n");
        sprintf(c0, "voltage at AIN0 = %f", v0_raw);
        return v0_raw;
    }
    if (channel == 2)
    {
        v1 = readAdc1Ss3();
        float v1_raw = (v1 * 3.3 / 4096);
        putsUart0("\r\n");
        sprintf(c1, "voltage at AIN1 = %f", v1_raw);
        return v1_raw;
    }

}
*/



void dc_cmd(void)
{
    mode = 2;
    uint32_t D, data;
    float V_OUT_A, V_OUT_B;
    uint8_t ch = getArgInt(1);
    float inp = getArgFloat(2);

    FSS = 0;
    LDAC = 1;
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");
    __asm("             NOP");

    if (ch == 1)
    {

        // V_OUT_A = ((inp - 5.0693) / (-5.0131)) + 0.015; //TF------y = -5.0131x + 5.0693     y = -0.1973x + 1.008

        V_OUT_A = -0.1973 * inp + 1.008 + 0.01;
        D = V_OUT_A * 4096 / 2.048;
        data = D | 12288; //1st 4 bits = 0011
        SSI1_DR_R = data;               // write command
        while (SSI1_SR_R & SSI_SR_BSY)
            ;
        LDAC = 0;

    }
    else if (ch == 2)
    {
        V_OUT_B = -0.1974 * inp + 1.0125; //TF------y = -5.0332x + 5.1028     y= -0.1974x + 1.0125
        D = (V_OUT_B * 4096 / 2.048);
        data = D | 45056;    ///1st 4 bits = 1011
        SSI1_DR_R = data;               // write command
        while (SSI1_SR_R & SSI_SR_BSY);
        LDAC = 0;

    }

    //int data = 0x37D0;   //0x2|0x1770;

}

void sine(void)                 // sine chan freq amp ofs
{
    p=1;
    uint8_t chan = getArgInt(1);
    float amp = getArgFloat(3);
        float ofs = getArgFloat(4);
        amp_1=amp;
        ofs_1=ofs;
    uint16_t i;

    if (chan == 1)
    {
        star = 1;

        freq_A = getArgFloat(2);
        del_phi_1 = freq_A * (pow(2, 32) / 100000);
        for (i = 0; i < 4096; i++)
        {

            LUT[0][i] = -394.6 * ((sin(2 * 3.1415 * i / 4096) * amp) + ofs)+ 2036;

        }

    }

    else if (chan == 2)
    {
        star = 2;
        freq_B = getArgFloat(2);
        del_phi_2 = freq_B * pow(2, 32) / 100000;
        for (i = 0; i < 4096; i++)
        {

            LUT[1][i] = -394.8 * ((sin(2 * 3.1415 * i / 4096) * amp) + ofs)
                    + 2025;
        }
    }

    //  TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    //NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    //TIMER1_CTL_R |= TIMER_CTL_TAEN;

}

void square(void)
{
    p=2;
    uint8_t chan = getArgInt(1);
    float amp = getArgFloat(3);
    float ofs = getArgFloat(4);
    float dc = getArgFloat(5);

    //  del_phi = freq * pow(2, 32) / 100000;

    float x;
    if (dc == 0)
    {
        dc = 50;
        x = 4096 - (dc * 41);
    }
    else
        x = (dc * 41);
    //  del_phi = freq * pow(2, 32) / 100000;
    uint16_t i;
    if (chan == 1)
    {
        freq_A = getArgFloat(2);
        del_phi_1 = freq_A * pow(2, 32) / 100000;
        star = 1;
        for (i = 0; i < 4096; i++)
        {
            if (i < x)
                LUT[0][i] = (-394.6 * (amp + ofs)) + 2036;
            if (i > x)
                LUT[0][i] = (-394.6 * ((-1) * amp) + ofs) + 2036;

        }

    }
    else if (chan == 2)
    {
        star = 2;
        freq_B = getArgFloat(2);
        del_phi_2 = freq_B * pow(2, 32) / 100000;
        for (i = 0; i < 4096; i++)
        {
            if (i < 2048)
                LUT[1][i] = (-394.8 * (amp + ofs)) + 2025;
            if (i > 2048)
                LUT[1][i] = (-394.6 * ((-1) * amp) + ofs) + 2025;

        }

    }

}

void triangle()
{
    p=3;
    uint8_t chan = getArgInt(1);
    // float freq = getArgFloat(2);
    float amp = getArgFloat(3);
    float ofs = getArgFloat(4);

    // del_phi = freq * pow(2, 32) / 100000;
    uint16_t i;
    if (chan == 1)
    {
        star = 1;
        freq_A = getArgFloat(2);
        del_phi_1 = freq_A * pow(2, 32) / 100000;
        for (i = 0; i < 4096; i++)
        {
            LUT[0][i] = -394.6 * (ofs+ ((2 * amp / 3.1415)* asin(sin(2 * 3.1415 * i / 4096)))) + 2036;

        }

    }
    else if (chan == 2)
    {
        freq_B = getArgFloat(2);
        star = 2;
        del_phi_2 = freq_B * pow(2, 32) / 100000;
        for (i = 0; i < 4096; i++)
        {
            LUT[1][i] = -394.8* (ofs+ ((2 * amp / 3.1415) * asin(sin(2 * 3.1415 * i / 4096)))) + 2025;

        }

    }

}

void sawtooth()
{
    p=4;
    uint8_t chan = getArgInt(1);
    float amp = getArgFloat(3);
    float ofs = getArgFloat(4);
    uint16_t i;
    if (chan == 1)
    {
        star = 1;
        freq_A = getArgFloat(2);
        del_phi_1 = freq_A * pow(2, 32) / 100000;
        for (i = 0; i < 4096; i++)
        {
            LUT[0][i] = (-394.6 * ((amp / 4095) * i + ofs) + 2036);

        }

    }
    else if (chan == 2)
    {
        star = 2;
        freq_B = getArgFloat(2);
        del_phi_2 = freq_B * pow(2, 32) / 100000;
        for (i = 0; i < 4096; i++)
        {
            LUT[1][i] = (-394.8 * ((amp / 4095) * i + ofs) + 2025);

        }
    }

}


void hilbert()
{

    uint16_t i;
    char* power = getArgString(1);
    if((strcmp("on",power))==0)
    {
    del_phi_2 = freq_A * pow(2, 32) / 100000;
            for (i = 0; i < 4096; i++)
            {

        LUT[1][i] = -394.8 * ((cos(2 * 3.1415 * i / 4096) * amp_1) + ofs_1)
                    + 2025;
            }


    }
    else if(strcmp("off",power)==0)
    {

        for (i = 0; i < 4096; i++)
                    {

                        LUT[1][i] = 2025;
                    }


    }


    }


void differential()
{


    uint16_t i;
    del_phi_2 = freq_A * pow(2, 32) / 100000;
    char* type = getArgString(1);
        if((strcmp("on",type))==0)
        {
    switch(p)
    {
    case 1:   //sine
    {

    for (i = 0; i < 4096; i++)
              {

                  LUT[1][i] = -394.8 * ((sin(2 * 3.1415 * i / 4096) *((-1)*amp_1)) + ofs_1)
                          + 2025;
              }
    break;
    }

    case 2:   //square
        {

        for (i = 0; i < 4096; i++)
                  {
            if (i < 2048)
                            LUT[1][i] = (-394.8 * (((-1)*amp_1) + ofs_1)) + 2025;
                        if (i > 2048)
                            LUT[1][i] = (-394.6 * (amp_1 + ofs_1)) + 2025;
                  }
        break;
        }

    case 3 :   //triangular
            {

            for (i = 0; i < 4096; i++)
                      {
                LUT[1][i] = -394.8* (ofs_1+ ((2 * (-1)*amp_1 / 3.1415) * asin(sin(2 * 3.1415 * i / 4096)))) + 2025;
                      }
            break;
            }

    case 4 :   //sawtooth
                {

                for (i = 0; i < 4096; i++)
                          {
                    LUT[1][i] = (-394.8 * ((((-1)*amp_1) / 4095) * i + ofs_1) + 2025);
                          }
                break;
                }




    }
        }

        else if((strcmp("off",type)==0))
                {
            for (i = 0; i < 4096; i++)
                                {

                                    LUT[1][i] = 2025;
                                }
                }

    }

/*

void gain()
{
    float f1,f2,gain,v_1,v_2;
    int step;
    uint16_t j;
    float i
    f1 = getArgFloat(1);
    f2 = getArgFloat(2);
    step = getArgInt(3);
    char c0[50],c1[50];

    for(j=0;j<4096;j++)
        LUT[0][j] = -394.6 * ((sin(2 * 3.1415 * j / 4096) * 4))+ 2036;


           for (i = f1; i <=f2; i=f1+step)
           {
               del_phi_1 = i* (pow(2, 32) / 100000);
               mode =1;

               waitMicrosecond(50000);

               v_1= readVoltage(1);
               v_2=readVoltage(2);
               gain=v_2/v_1;
               sprintf(c1, "freq = %f", i);
               putsUart0(c1);
               putsUart0("\t");


               sprintf(c0, "gain = %f", gain);
               putsUart0(c0);
               putsUart0(" \r\n");


           }


    }

*/


void timer1Isr()
{
    if (mode == 1)
    {
        if (flag == 1)
        {
            val--;
            if (val == 0)
            {
                mode = 0;
            }
        }

        uint8_t ch = getArgInt(1);
        uint16_t value_0, value_1;

        phi_1 = phi_1 + del_phi_1;
        value_0 = LUT[0][phi_1 >> 20]; //channel 1/A
        SSI1_DR_R = value_0 + 12288;

        phi_2 = phi_2 + del_phi_2;
        value_1 = LUT[1][phi_2 >> 20];
        SSI1_DR_R = value_1 + 45056;

        LDAC = 0;

    }
    else if (mode == 0)
    {
        SSI1_DR_R = 2036 | 12288;
        SSI1_DR_R = 2025 | 45056;

    }
    else if (mode == 2)
    {

    }
    TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag

}

void run()
{
    mode = 1;

    TIMER1_IMR_R = TIMER_IMR_TATOIM;                 // turn-on interrupts
    NVIC_EN0_R |= 1 << (INT_TIMER1A - 16);     // turn-on interrupt 37 (TIMER1A)
    TIMER1_CTL_R |= TIMER_CTL_TAEN;
}

void stop()
{
    mode = 0;

}

void cycles()
{
    uint8_t c = getArgInt(1);
    flag = 1;
    if (freq_A != 0)
    {
        val = 100000 * c / freq_A;
    }

    if (freq_B != 0)
    {
        val = 100000 * c / freq_B;
    }

}

void sys_reset(void)
{

    //  putsUart0("Reset \r\n");
    //waitMicrosecond(1000000);
    //NVIC_APINT_R = 0x04 | (0x05FA << 16);

    __asm("    .global _c_int00\n"
            "    b.w     _c_int00");

}

void cmd()
{
    if (isCommand("reset", 0))
        sys_reset();
    else if (isCommand("voltage", 1))
        getVoltage();
    else if (isCommand("dc", 2))
        dc_cmd();
    else if (isCommand("sine", 4))
        sine();
    else if (isCommand("run", 0))
        run();
    else if (isCommand("stop", 0))
        stop();
    else if (isCommand("square", 4) || isCommand("square", 5))
        square();
    else if (isCommand("triangle", 4))
        triangle();
    else if (isCommand("sawtooth", 4))
        sawtooth();
    else if (isCommand("cycles", 1))
        cycles();
    else if (isCommand("hilbert", 1))
            hilbert();
    else if (isCommand("differential", 1))
                differential();
  //  else if (isCommand("gain", 3))
    //                gain();


    else
        putsUart0("Invalid Command");

}

void main(void)
{
    initHw();
    //uint8_t i, j;
    //char* test[80];
    flash_led();

       /*for (i = 0; i <= strlen(str); i++)
     {
     for (j = 0; j <= strlen(str); j++)
     {
     putcUart0(ans[i][j]);
     }
     putsUart0("\r\n");
     }
     */
    /*putsUart0("verb + arguments count = ");
     putsUart0((char) (argCount));
     putsUart0("number of arguments is ");
     putsUart0((char) (argCount-1));
     */


    while (1)
    {
        putsUart0("\r\n");
        putsUart0("Enter String - ");
        getString();
        parseString();
        putsUart0("\r\n");
        cmd();


    }


}
