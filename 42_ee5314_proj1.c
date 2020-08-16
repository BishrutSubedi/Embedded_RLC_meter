// Low cost RLC meter 
// Subedi Bishrut

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// Red Backlight LED:
//   PB5 drives an NPN transistor that powers the red LED
// Green Backlight LED:
//   PE4 drives an NPN transistor that powers the green LED
// Blue Backlight LED:
//   PE5 drives an NPN transistor that powers the blue LED
// Red LED:
//   PF1 drives an NPN transistor that powers the red LED
// Green LED:
//   PF3 drives an NPN transistor that powers the green LED
// Pushbutton:
//   SW1 pulls pin PF4 low (internal pull-up is used)
// UART Interface:
//   U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
//   Configured to 115,200 baud, 8N1

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "tm4c123gh6pm.h"

#include <stdlib.h>

#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) //PF1
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) //PF3
#define PUSH_BUTTON  (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4

#define MEAS_LR     (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) //PD2
#define MEAS_C      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) //PD3
#define HIGHSIDE_R  (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define LOWSIDE_R   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //PE2
#define INTEGRATE   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 3*4))) //PE3

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// This code does not have error in part 2
// variables max_Char, str[81], and count moved as global from main
// also str[81] used instead of str[max_Count + 1], took care of warning, str declared but not used(while inside main)

uint8_t max_Char = 80;
char str[81]; // warning declared but never used
uint8_t count = 0; // is #no of char in buffer, where is my buffer here ? what is buffer ?
uint8_t counter;

// for part 3
//uint8_t argc;
int argc;
uint8_t pos []; //can be char/int
char type[]; //can be char/int // find whats the difference
char str_1[82]; //dummy string of delimeter, alpha, etc // size later goes null don't matter
int arg1, arg2;
int number; // dont' know if necessary for function get string check
int number_1; // testing number
int arg;
int time_value;
int time;
double raw;
double voltage;
char temp[25];
int raw_1;
// Blocking function that returns only when SW1 is pressed
void waitMicrosecond(uint32_t us)
{
    __asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*2 (speculative, so P=1)
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             NOP");                  // 1
    __asm("             B    WMS_LOOP0");       // 1*2 (speculative, so P=1)
    __asm("WMS_DONE0:");                        // ---
                                                // 40 clocks/us + error
}

void configADC(){ // PE3
// Configure AN0 as an analog input
SYSCTL_RCGCADC_R |= 1;                           // turn on ADC module 0 clocking
GPIO_PORTE_AFSEL_R |= 0x01;                      // select alternative functions for AN0 (PE0)
GPIO_PORTE_DEN_R &= ~0x01;                       // turn off digital operation on pin PE0
GPIO_PORTE_AMSEL_R |= 0x01;                      // turn on analog operation on pin PE0
ADC0_CC_R = ADC_CC_CS_SYSPLL;                    // select PLL as the time base (not needed, since default value)
ADC0_ACTSS_R &= ~ADC_ACTSS_ASEN3;                // disable sample sequencer 3 (SS3) for programming
ADC0_EMUX_R = ADC_EMUX_EM3_PROCESSOR;            // select SS3 bit in ADCPSSI as trigger
ADC0_SSMUX3_R = 3;                              // set first sample to AN0 (What it does?) check adadi 0 thiyo
ADC0_SSCTL3_R = ADC_SSCTL3_END0;                 // mark first sample as the end
ADC0_ACTSS_R |= ADC_ACTSS_ASEN3;                 // enable SS3 for operation
}

 int16_t readAdc0Ss3()
 {
     ADC0_PSSI_R |= ADC_PSSI_SS3;                     // set start bit
     while (ADC0_ACTSS_R & ADC_ACTSS_BUSY);           // wait until SS3 is not busy
     return ADC0_SSFIFO3_R;                           // get single result from the FIFO
 }

void blinkey(){
    waitMicrosecond(500000);
      RED_LED = 1;
      waitMicrosecond(500000);
      RED_LED = 0;
}

void waitPbPress() {
    while (PUSH_BUTTON);
}

// Initialize Hardware

void initHw() {
    // Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Set GPIO ports to use APB (not needed since default configuration -- for clarity)
    // Note UART on port A must use APB
    SYSCTL_GPIOHBCTL_R = 0;

    // Enable GPIO port A and F peripherals
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOF |SYSCTL_RCGC2_GPIOD | SYSCTL_RCGC2_GPIOE |SYSCTL_RCGC2_GPIOC; //XTRA enable D,E,C

    // Configure LED and pushbutton pins
    GPIO_PORTF_DIR_R = 0x0A; // bits 1 and 3 are outputs, other pins are inputs
    GPIO_PORTF_DR2R_R = 0x0A; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R = 0x1A; // enable LEDs and pushbuttons
    GPIO_PORTF_PUR_R = 0x10; // enable internal pull-up for push button

    GPIO_PORTD_DIR_R = 0x0C; // GPIO port 2 and 3 //ADDED
    GPIO_PORTD_DR2R_R = 0x0C;
    GPIO_PORTD_DEN_R = 0x0C;
    //GPIO_PORTD_PUR_R = 0x0C; // no pull up

    GPIO_PORTE_DIR_R = 0x0E; // GPIO PORTE 1, 2 and 3 // ADDED
    GPIO_PORTE_DR2R_R = 0x0E;
    GPIO_PORTE_DEN_R = 0x0E;


    // Configure UART0 pins
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0; // turn-on UART0, leave other uarts in same status
    GPIO_PORTA_DEN_R |= 3; // default, added for clarity
    GPIO_PORTA_AFSEL_R |= 3; // default, added for clarity
    GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

    // Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
    UART0_CTL_R = 0; // turn-off UART0 to allow safe programming
    UART0_CC_R = UART_CC_CS_SYSCLK; // use system clock (40 MHz)
    UART0_IBRD_R = 21; // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART0_FBRD_R = 45; // round(fract(r)*64)=45
    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module

    // ADDED ON INIT HW
    MEAS_LR = 0 ;
    MEAS_C = 0;
    HIGHSIDE_R = 0;
    LOWSIDE_R = 0;
    INTEGRATE = 0;
    RED_LED = 0;
    GREEN_LED = 0;

    // Configuring comparator0 pins
    SYSCTL_RCGCACMP_R = SYSCTL_RCGCACMP_R0; //0x00000001; // Enabling analog comparator clock
    GPIO_PORTC_AFSEL_R |= 0x08;                         // turn on alternate function on PC7
    GPIO_PORTC_DEN_R &= ~0x08;                          // turn off digital operation on pin PC7
    GPIO_PORTC_AMSEL_R |= 0x08;                         // turn on analog operation on pin PE3
    GPIO_PORTC_DIR_R &= ~0x08;                          // PORT PC7 as input to comparator

    // configuring comparator0 as specification
    //COMP_O_ACMIS = COMP_ACMIS_IN0;
   // COMP_ACREFCTL = COMP_ACREFCTL_EN | COMP_ACREFCTL_VREF_M; //0x0000020F;// setting EN = 1 and vref = 3.3V 0b001000001111.
    COMP_ACREFCTL_R = 0x000020F;
    // COMP_O_ACREFCTL = // try using macros
    COMP_ACCTL0_R = 0x00004DA; //COMP_ACCTL0_CINV | COMP_ACCTL0_ISEN_RISE |COMP_ACCTL0_ISLVAL| COMP_ACCTL0_TSEN_RISE | COMP_ACCTL0_TSLVAL;                          // setting ACCTL0 register
   // COMP_O_ACMIS = 0x01;
   // COMP_O_ACRIS = 0x01;
   //COMP_ACINTEN = 0x01; // enable interrupt, if happens try to process it.
    waitMicrosecond(10); //Data sheet
   NVIC_EN0_R |= 1<<(41-16); // interrupt


}

// Blocking function that writes a serial character when the UART buffer is not full

void putcUart0(char c) { // code does not leave line 90 until fifo not full
    while (UART0_FR_R & UART_FR_TXFF); // before you write on line 91 wait till you have space on FIFO
    UART0_DR_R = c; // how fast can you write , 16 at a time
} // line 91 puts character on fifo and returns the fifo, don't write on buffer until there is room for it.

// Blocking function that writes a string when the UART buffer is not full

void putsUart0(char* str) {
    uint8_t i;
    for (i = 0; i < strlen(str); i++)
        putcUart0(str[i]);
}

// Blocking function that returns with serial data once the buffer is not empty

char getcUart0() {

    while (UART0_FR_R & UART_FR_RXFE); // before reading line 106 make sure receiver frame buffer (FIFO) is not empty
    return UART0_DR_R & 0xFF; // reads the data register
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void parse_string(char *str) {
    str_1[0] = 'd'; // dummy string with first delimeter representation
    uint8_t j, k, m, n = 0;
    //uint8_t counter = 0; // counter defined as uint8_t check it and redefine as int
    counter = 0;
    // create global variable  of argc; pos[]; type[];
    while (1) {
        for (j = 0; j < count; j++) { // go through string to analyze all char of string

            if (str[j] >= 48 && str[j] <= 57) { // condition of number
                str_1[j + 1] = 'n'; // suedo for delimeter

            } else if (str[j] >= 65 && str[j] <= 122) { // condition of letter
                str_1[j + 1] = 'l';

            } else { // condition for delimeter
                str_1[j + 1] = 'd';

            }

        } // END FOR LOOP

       // putsUart0("printing dummy str1");
        putsUart0("\r\n");
       // putsUart0(str_1);
       // putsUart0("\r\n");

        for (k = 0; k < count; k++) {

            if (str_1[k] == 'd' && str_1[k + 1] == 'l') {
                pos[counter] = k;
                type[counter] = 'a'; // delimeter to letter
                counter++;
            } else if (str_1[k] == 'd' && str_1[k + 1] == 'n') {
                pos[counter] = k;
                type[counter] = 'n'; // delimeter to number
                counter++;
            } else if (str_1[k] == 'l' && str_1[k + 1] == 'n') {
                pos[counter] = k;
                type[counter] = 'n'; // letter to number
                counter++;
            } else {
                /*do nothing condition*/
            }
            /*check if you could end position array or type array with empty element or not*/
        }

        argc = counter; // last null check subtracted (counter-1) or what ?
        arg = counter - 1; // can you put arg here, and update global variable, its a function or what ?

        for (m = 1; m < count + 1; m++) { // NUlling delimeter
            if (str_1[m] == 'd') { //convert every delimeter to null // actual str or suedo string
                str[m - 1] = '\0';
            }
        }
        break;
    }
   // putsUart0("printing important stuff");
    //putsUart0("\r\n");

    // checking step only
    for (n = 0; n < argc; n++) { // argc replaced  by counter
        putsUart0(&str[pos[n]]);
        putsUart0("\r\n");
        // putsUart0(13);
    }

    for (n = 0; n < count + 1; n++) { // read it
        str_1[n] = '\0'; // empty out str_1 in case if new str has less index
    }


    return;

} //END FUNCTION PARSE_STRING

int getvalue(int arg) { // check this function
    number = 0; // clearing number
    if (type[arg] == 'n') { // if or while
        number = atoi(&str[pos[arg]]); //pos[arg] changed to 1.number still 0 num 1 changed to 12.
    } // check the number as global or local variable in code composer studio varible list
    return number;
}

bool is_Command(char *str1, int arg) { // pass string and minimum number of argument here
    // so is it supposed to find argument by itself or not
    // is command ma verb and argument pass garne
    // argument pathaunu parne ma, argument lai overwrite gareko cha
    bool valid = false; // by default valid lai false banaune

    //if ((strcmp("set", &str[pos[0]]) == 0) && (arg == 2)) {  // argc-1 for funtion to execute argc = 3
    if ((strcmp(str1, &str[pos[0]]) == 0) && (arg == 2)) { // argc-1 for funtion to execute argc = 3
        valid = true;
    } else if ((strcmp(str1, &str[pos[0]]) == 0) && (arg == 1)) { //argc-1 // for funtion to execute argc = 3
        valid = true;
    }
    else {}

    return valid;
}

void setTimerMode() // doctor losh
{
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;     // turn-on timer
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;                // turn-off counter before reconfiguring
    WTIMER5_CFG_R = 4;                               // configure as 32-bit counter (A only)
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER5_CTL_R = TIMER_CTL_TAEVENT_POS;           // measure time from positive edge to positive edge
    //WTIMER5_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts // on an edge
    WTIMER5_TAV_R = 0;                               // zero counter for first period //
   // WTIMER5_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter // turn on timer // on fn to calculate time
   // NVIC_EN3_R |= 1 << (INT_WTIMER5A-16-96);         // turn-on interrupt 120 (WTIMER5A) // turn on nvic controller for timer 20 or what ?line 182
}

void compIsr(){

    time_value = WTIMER5_TAV_R; // CHECK IF YOU NEED MICROSECOND
    raw = readAdc0Ss3();
    voltage = ((((double)raw / 4096 )*3.3 - 0.02 ));// this voltage overwrites blocking function
    sprintf(temp, "Voltage comp : %2.2lf", voltage);
    putsUart0(temp);
    putsUart0("\n\r");
    waitMicrosecond(500000);
    time = time_value / 40;
    WTIMER5_TAV_R = 0; // clearing timer

    //COMP_ACINTEN_R &= ~0x01; // uncomment gareko DISABLEe interrupt, if happens try to process it.
    COMP_ACMIS_R = 0x01; // comparator 0 is bit1
    //COMP_ACINTEN_R = 0x00; // uncomment gareko DISABLEe interrupt, if happens try to process it.

    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    // resistance = t/c;
    //init comparator
    //get the timer value

}


int main(void) {
    // Initialize hardware
    initHw();

    configADC();

    // Toggle LED's
    blinkey();

    // Display greetings
    putsUart0("------LCR Meter--------\r\n");
    putsUart0("-----------------------\r\n");
    putsUart0("-----Enter Command: \r\n");

    while (1) {
        count = 0;
        // str[80] = { '\0' }; // Nulling out all string
        while (1) { //str[count] != '\0'
            char c = getcUart0();

            if (c != 8 && c != 13 && c >= 32) { // CHECKING BACKSPACE AND CR
                if (c >= 65 & c <= 90) {
                    c = c + 32; // changing capital to small letter
                }
                // str[count++] = c; //check this logic for where bs puts character (by printing ?) put on 1 or 0; ask Dr. Losh
                str [count] = c;
                count++;
                if (count == max_Char) {
                    str[count] = '\0';
                    break;
                }
                // break; //which one does it break (find other ways to break like flag // check to create flag on stack exchanges)
            }

            if (c == 8) { // back space
                if (count > 0) {
                    count--;
                }
            }

            if (c == 13) { //carriage return
                str[count] = '\0'; // null terminator, marks end of string in C.
                break; // where does this break go to ?
            }

        } //END INSIDE WHILE LOOP
        // putsUart0(str); // checking the input string
        //putsUart0("\r\n");

        parse_string(str); // calling function parse string to do step 3.


        double resistance;
        double resistance_1;
        char temp_1[25];
        double capacitance;
        double capacitance_1;
        double inductor;
        double inductor_1;
        double inductor_2;
        double ESR;

        // STEP 4
        if (is_Command("set", 2)) { //if(true/false) // not working always // send argc-1

            arg1 = getvalue(1); // what if there are three arguments ? we give arguments as user input // not auto
            arg2 = getvalue(2); // How to print arg2, can't see in global variable as well
        }

        else if (is_Command("measc", 1)) { //MeasC on
            if ((strcmp("on", &str[pos[1]]) == 0)) {
                RED_LED = 1;
                MEAS_C = 1;
            } else if ((strcmp("off", &str[pos[1]]) == 0)) {
                RED_LED = 0;
                MEAS_C = 0;
            }
        }

        else if (is_Command("measlr", 1)) { //MeasC on
                    if ((strcmp("on", &str[pos[1]]) == 0)) {
                        GREEN_LED = 1;
                        MEAS_LR = 1;
                    } else if ((strcmp("off", &str[pos[1]]) == 0)) {
                        GREEN_LED = 0;
                        MEAS_LR = 0;
                    }
                }
        else if (is_Command("highsider", 1)) { //MeasC on
                    if ((strcmp("on", &str[pos[1]]) == 0)) {
                        RED_LED = 1;
                        HIGHSIDE_R = 1;
                    } else if ((strcmp("off", &str[pos[1]]) == 0)) {
                        RED_LED = 0;
                        HIGHSIDE_R = 0;
                    }
                }
        else if (is_Command("lowsider", 1)) { //MeasC on
                    if ((strcmp("on", &str[pos[1]]) == 0)) {
                        GREEN_LED = 1;
                        LOWSIDE_R = 1;
                    } else if ((strcmp("off", &str[pos[1]]) == 0)) {
                        GREEN_LED = 0;
                        LOWSIDE_R = 0;
                    }
                }
        else if (is_Command("integrate", 1)) { //MeasC on
                    if ((strcmp("on", &str[pos[1]]) == 0)) {
                        GREEN_LED = 1;
                        INTEGRATE = 1;
                    } else if ((strcmp("off", &str[pos[1]]) == 0)) {
                        GREEN_LED = 0;
                        INTEGRATE = 0;
                    }
                }
        else if (is_Command("allzero", 1)) { //MeasC on
            MEAS_C = 0;
            MEAS_LR = 0;
            HIGHSIDE_R = 0;
            LOWSIDE_R = 0;
            INTEGRATE = 0;
            RED_LED = 0;
            GREEN_LED = 0;
                }
        else if (is_Command("voltage", 1)) { //MeasC on

            MEAS_LR = 0;
            MEAS_C = 1;
            RED_LED = 1;
            raw = readAdc0Ss3();
            voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);

            sprintf(temp, "Raw : %lf", raw);
            putsUart0(temp);
            putsUart0("\n\r");

            sprintf(temp, "Voltage : %2.2lf", voltage);
            putsUart0(temp);
            waitMicrosecond(500000);
                        }
        else if (is_Command("reset", 1)) { //MeasC on
            //NVIC_APINT_R = 0x0004|(0x05FA << 16);
                  NVIC_APINT_R = 0x05FA0004;
                }

        else if (is_Command("resistor", 1)) { //MeasC on
                        setTimerMode();
                       // discharge cap
                       LOWSIDE_R = 1;
                       MEAS_LR = 0;
                       MEAS_C = 0;

                       INTEGRATE = 1;
                       HIGHSIDE_R = 0;
                     //  waitMicrosecond(2000000);//discharge time
                       voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);

                       while(voltage > 0){
                       raw = readAdc0Ss3();
                       voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                       }
                       sprintf(temp, "Voltage : %2.2lf", voltage);
                       putsUart0(temp);
                       putsUart0("\n\r");

                       COMP_ACINTEN_R |= 0x01; // enable interrupt, if happens try to process it.
                       WTIMER5_TAV_R = 0;
                       // Charging capacitor
                       MEAS_LR = 1; // DIFFN
                       LOWSIDE_R = 0; // DIFFN
                       WTIMER5_CTL_R |= TIMER_CTL_TAEN;
                     //  INTEGRATE = 1;
                     //  HIGHSIDE_R = 0;
                      // waitMicrosecond(2000000);
                       while(voltage < 2.469){//2.469
                           raw = readAdc0Ss3();
                           voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                       } // voltage updated by interrupt
                       //waitMicrosecond(20000000);

                       sprintf(temp, "time_value_r: %d", time_value); // putting time value in
                       putsUart0(temp);
                       putsUart0("\n\r");
                       waitMicrosecond(500000);

                       sprintf(temp_1, "time_r: %d", time); // putting time value in
                       putsUart0(temp_1);
                       putsUart0("\n\r");
                       waitMicrosecond(500000);
                       // HERE GET IF CONDITION FOR LARGER AND SMALL RES
                       // SEPERATE THEM BASED ON TAU OR TIME

                       if (time < 75000) { //80000
                       resistance = 0.8015 * time - 11.1; //
                       //resistance = 0.7999 * time - 12.3;
                       sprintf(temp, "Resistor small: %0.2lf", resistance); // putting time value in
                       putsUart0(temp);
                       putsUart0("\n\r");
                       }
                       else {
                       resistance_1 = 1.2978 * time - 40246;
                       sprintf(temp, "Resistor large: %0.2lf", resistance_1); // putting time value in
                       putsUart0(temp);
                       putsUart0("\n\r");
                       }

                       putsUart0("-----------------------\r\n");
                       putsUart0("-----------------------\r\n");
                       putsUart0("Enter Command: \r\n");

                      }

        else if (is_Command("capacitor", 1)) { //MeasC on
                         setTimerMode();
                         // pic fail safe mechanism
                         // disable interrupt
                         // clear flag
                       // COMP_ACINTEN_R = 0x00;
                       // COMP_ACMIS_R = 0x01;

                        // discharge cap
                        MEAS_LR = 0; // 0
                        HIGHSIDE_R = 0; // 0
                        INTEGRATE = 1; // changed from 1 to 0
                        LOWSIDE_R = 1; //1
                        MEAS_C = 1; //1
                        putsUart0("Discharging:");

                        raw = readAdc0Ss3();
                        voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);

                        while(voltage > 0){
                            raw = readAdc0Ss3();
                            voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                        }

                        sprintf(temp, "Voltage : %2.2lf", voltage);
                        putsUart0(temp);
                        putsUart0("\n\r");

                        COMP_ACINTEN_R = 0x01; // enable interrupt, if happens try to process it.
                        WTIMER5_TAV_R = 0;

                        putsUart0("Charging:");
                        putsUart0("\n\r");
                        // Charging capacitor
                        LOWSIDE_R = 0; // DIFFN //0
                        INTEGRATE = 0; //0
                        HIGHSIDE_R = 1; //1
                        MEAS_C = 1; //1
                        WTIMER5_CTL_R |= TIMER_CTL_TAEN;

                       // blocking function while charging

                        while(voltage < 2.469){//Never reaches this voltage 2.469 for 100uF stuck at 2.02// cuz wrong polarity
                            raw = readAdc0Ss3();
                            voltage = ( ( ((double)raw / 4096 )*3.3 -0.02 ) );
                            sprintf(temp, "Voltage blocking : %2.2lf", voltage);
                            putsUart0(temp);
                            putsUart0("\n\r");
                            waitMicrosecond(500000);
                        }

                        //waitMicrosecond(20000000); if delay read correct value of 33uF

                        sprintf(temp, "Tau: %d", time_value); // putting time value in
                        putsUart0(temp);
                        putsUart0("\n\r");
                        waitMicrosecond(500000);

                        sprintf(temp_1, "time[us]: %d", time); // putting time value in
                        putsUart0(temp_1);
                        putsUart0("\n\r");
                        waitMicrosecond(500000);

                        if(time < 345000){
                        capacitance = 0.000006534 * time + 0.1185;
                        sprintf(temp, "Capacitance linear[uF]: %0.5lf", capacitance); // putting time value in
                        putsUart0(temp);
                        putsUart0("\n\r");
                        }
                        else {
                        sprintf(temp, "Capacitance poly[uF]: %0.5lf", capacitance_1); // putting time value in
                        capacitance_1 = 0.000006891 * time  - 0.0000000000000524 * time * time + 0.0010 ;
                        putsUart0(temp);
                        putsUart0("\n\r");
                        }
                        putsUart0("-----------------------\r\n");
                        putsUart0("-----------------------\r\n");
                        putsUart0("Enter Command: \r\n");

                       }

        else if (is_Command("inductor", 1)) { //MeasC on
                         setTimerMode();
                        // discharge cap
                        MEAS_LR = 0;
                        HIGHSIDE_R = 0;
                        INTEGRATE = 1;
                        MEAS_C = 0;
                        LOWSIDE_R = 1;

                        putsUart0("Discharging:");
                        putsUart0("\n\r");
                      //  waitMicrosecond(1000000);//discharge time 5 to 1

                        voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                        while(voltage > 0){
                            raw = readAdc0Ss3();
                            voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                        }

                       // raw = readAdc0Ss3();
                       //voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                        sprintf(temp, "Voltage : %2.2lf", voltage);
                        putsUart0(temp);
                        putsUart0("\n\r");

                        COMP_ACINTEN_R |= 0x01; // enable interrupt, if happens try to process it.
                        WTIMER5_TAV_R = 0;

                        putsUart0("Charging:");
                        putsUart0("\n\r");
                        // Charging inductor
                        LOWSIDE_R = 1;
                        MEAS_LR = 1;
                        INTEGRATE = 0;
                        WTIMER5_CTL_R |= TIMER_CTL_TAEN;
                        HIGHSIDE_R = 0;
                        MEAS_C = 0;
                        //WTIMER5_CTL_R |= TIMER_CTL_TAEN;

                        //waitMicrosecond(5000000); // charging time
                        while(voltage < 2.469){
                            raw = readAdc0Ss3();
                            voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                        }

                        sprintf(temp, "tau: %d", time_value); // putting time value in
                        putsUart0(temp);
                        putsUart0("\n\r");
                        waitMicrosecond(500000);

                        sprintf(temp_1, "time[us]: %d", time); // putting time value in
                        putsUart0(temp_1);
                        putsUart0("\n\r");
                        waitMicrosecond(500000);


                        inductor = 0.6596 * time_value + 0.5323;
                        inductor_1 = 0.6601 * time_value + 0.3068;
                        //inductor = 0.0000006598 * time_value + 0.0000005323;
                       // inductor_2 = 0.55701 * log(time_value) - 1.9600;
                       // sprintf(temp, "Inductor No(0,0)[uH]: %0.3lf", inductor); // putting time value in
                       // putsUart0(temp);
                        //putsUart0("\n\r");
                        sprintf(temp, "Inductor(0,0)[uH]: %0.3lf", inductor_1); // putting time value in
                        putsUart0(temp);
                        putsUart0("\n\r");
                        sprintf(temp, "Inductor log[mH]: %0.3lf", inductor_2); // putting time value in
                        putsUart0(temp);


                        putsUart0("\n\r");

                        putsUart0("-----------------------\r\n");
                        putsUart0("-----------------------\r\n");
                        putsUart0("Enter Command: \r\n");

                       }

        else if (is_Command("esr", 1)) { //MeasC on
                                setTimerMode();
                               // discharge cap 1uF
                               MEAS_LR = 0;
                               HIGHSIDE_R = 0;
                               INTEGRATE = 1;
                               MEAS_C = 0;
                               LOWSIDE_R = 1;

                               putsUart0("Discharging:");
                               putsUart0("\n\r");
                               voltage = ( ( ((double)raw / 4096 )*3.3 -.02 ));
                               while(voltage > 0){
                                   raw = readAdc0Ss3();
                                   voltage = ( ( ((double)raw / 4096 )*3.3 - 0.02 ));
                               }

                              // raw = readAdc0Ss3();
                              //voltage = ( ( ((double)raw / 4096 )*3.3 ) - 0.02);
                               sprintf(temp, "Voltage : %2.2lf", voltage);
                               putsUart0(temp);
                               putsUart0("\n\r");

                               //Don't have to enable interrupt here, just meas  voltage
                               //COMP_ACINTEN_R = 0x01; // enable interrupt, if happens try to process it.
                              // WTIMER5_TAV_R = 0;

                               putsUart0("Charging:");
                               putsUart0("\n\r");
                               // Charging inductor(ESR)
                               LOWSIDE_R = 1;
                               INTEGRATE = 0;
                               MEAS_LR = 1;
                               HIGHSIDE_R = 0;
                               MEAS_C = 0;
                              // WTIMER5_CTL_R |= TIMER_CTL_TAEN;

                               //waitMicrosecond(5000000); // charging time
                               while(1){
                                   raw = readAdc0Ss3();
                                   waitMicrosecond(2000000);
                                   raw_1 = readAdc0Ss3();

                                   while ((raw_1 - raw) != 0){ // blocking fn
                                       raw = readAdc0Ss3();
                                       waitMicrosecond(2000000);
                                       raw_1 = readAdc0Ss3();
                                   }
                                   voltage = ((((double)raw_1 / 4096 )*3.3 )); // Voltage on DUT2
                                   break;
                               }

                               sprintf(temp, "Voltage : %2.2lf", voltage);
                               putsUart0(temp);
                               putsUart0("\n\r");

                               sprintf(temp, "raw: %d", raw_1); // putting time value in
                               putsUart0(temp);
                               putsUart0("\n\r");
                               waitMicrosecond(500000);


                               ESR = (( 3.1  * 33.0 ) / voltage) - 33.0;// formula voltage divider
                               sprintf(temp, "ESR formula[mH]: %0.2lf ohm", ESR); // putting time value in
                               putsUart0(temp);
                               putsUart0("\n\r");
                              // sprintf(temp, "Inductor log[mH]: %0.5lf", inductor); // putting time value in
                             //  putsUart0(temp);

                               putsUart0("\n\r");

                               putsUart0("-----------------------\r\n");
                               putsUart0("-----------------------\r\n");
                               putsUart0("Enter Command: \r\n");

                              }
        else {
            putsUart0("-----------------------\r\n");
            putsUart0("Invalid Command: \r\n");
            waitMicrosecond(500000);
            putsUart0("Enter Command again: \r\n");
            putsUart0("-----------------------\r\n");
            //putsUart0("-----------------------\r\n");
        }



    } // END OUTSIDE WHILE LOOP

} // END MAIN


/*
 * DON'T FORGET TO CLEAR THE STRING AT THE END OF EACH LOOP
 * OR ELSE NEXT LOOP HAS SAME STRING MAY BE NOT, DELIMETER THERE
 * WHY POSITION[I] IS SAME THEN. CLEAR POSITION[I].
 */


