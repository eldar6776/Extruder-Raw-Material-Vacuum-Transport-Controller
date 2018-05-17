/********************************************************************
 *
 *       Displej gresaka kontrolera transporta materijala linije 5
 *
 *                    Firmware versija 1.00
 *
 *********************************************************************
 *  Ime fajla:      main.c
 *
 *  Procesor:       PIC12F675
 *
 *  Kompajler:      Microchip XC8 v1,13
 *
 *  IDE:            MPLAB X IDE v2.10 (java 1.7.0_25)
 *
 *  Datum:          17. jun 2014.
 *
 *  Autor:          eldar6776@hotmail.com
 *
 * Napomena:        flash verzija pic microkontrolera koristena za
 *                  debug koda, radni mikrokontroler je PIC12C508
 *
 ******************************************************************************/
//
/** U K L J U C E N I  ********************************************************/
//
#include <xc.h>
#include "typedefs.h"
//
/** K O N F I G U R A C I J A    P R O C E S O R A  ***************************/
//
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on GP4/OSC2/CLKOUT pin, I/O function on GP5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON       // Power-Up Timer Enable bit (PWRT enabled)
#pragma config MCLRE = OFF      // GP3/MCLR pin function select (GP3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
//
/** D E F I N I C I J E  ******************************************************/
//
#define _XTAL_FREQ 4000000 // oscillator frequency for delay functions
//
//-------------------------------- GPIO
//
#define SH_CLK      GPIO0
#define SH_DATA     GPIO1
#define MUX_C       GPIO2
#define MUX_IN      GPIO3
#define SH_LATCH    GPIO4
#define DSP_10      GPIO5
//
#define GPIO_Init() (CMCON = 0x07, ANSEL = 0x00, TRISIO = 0b00001000, GPIO = 0x00)
#define TMR0_Init() (OPTION_REG = 0x87, TMR0 = 0)
//
/** V A R I J A B L E  ********************************************************/
//
unsigned char digit;
unsigned char tmr_cnt;
unsigned char tmp_cnt;
//
/** K O N S T A N T E *********************************************************/
//
#define SelectDisplay10Input1()     (SH_CLK = HIGH, SH_DATA = HIGH, MUX_C = HIGH)
#define SelectDisplay10Input2()     (SH_CLK = LOW, SH_DATA = HIGH, MUX_C = HIGH)
#define SelectDisplay10Input4()     (SH_CLK = HIGH, SH_DATA = LOW, MUX_C = HIGH)
#define SelectDisplay10Input8()     (SH_CLK = LOW, SH_DATA = LOW, MUX_C = HIGH)
#define SelectDisplay1Input1()      (SH_CLK = HIGH, SH_DATA = HIGH, MUX_C = LOW)
#define SelectDisplay1Input2()      (SH_CLK = LOW, SH_DATA = HIGH, MUX_C = LOW)
#define SelectDisplay1Input4()      (SH_CLK = HIGH, SH_DATA = LOW, MUX_C = LOW)
#define SelectDisplay1Input8()      (SH_CLK = LOW, SH_DATA = LOW, MUX_C = LOW)
#define DIGIT_0_MASK                (0b00111111)
#define DIGIT_1_MASK                (0b00000110)
#define DIGIT_2_MASK                (0b01011011)
#define DIGIT_3_MASK                (0b01001111)
#define DIGIT_4_MASK                (0b01100110)
#define DIGIT_5_MASK                (0b01101101)
#define DIGIT_6_MASK                (0b01111101)
#define DIGIT_7_MASK                (0b00000111)
#define DIGIT_8_MASK                (0b01111111)
#define DIGIT_9_MASK                (0b01101111)
#define CHAR_A_MASK                 (0b01110111)
#define CHAR_B_MASK                 (0b01111100)
#define CHAR_C_MASK                 (0b00111001)
#define CHAR_D_MASK                 (0b01011110)
#define CHAR_E_MASK                 (0b01111001)
#define CHAR_F_MASK                 (0b01110001)
//
/** M A R K E R I  ************************************************************/
//
BYTE mux_input;
#define DSP_10_IN_1     mux_input.b0
#define DSP_10_IN_2     mux_input.b1
#define DSP_10_IN_4     mux_input.b2
#define DSP_10_IN_8     mux_input.b3
#define DSP_1_IN_1      mux_input.b4
#define DSP_1_IN_2      mux_input.b5
#define DSP_1_IN_4      mux_input.b6
#define DSP_1_IN_8      mux_input.b7
//
BYTE shift_data;
#define SEG_A           shift_data.b0
#define SEG_B           shift_data.b1
#define SEG_C           shift_data.b2
#define SEG_D           shift_data.b3
#define SEG_E           shift_data.b4
#define SEG_F           shift_data.b5
#define SEG_G           shift_data.b6
#define DSP_1           shift_data.b7
//
/******************************************************************************/
/***                                                                ***********/
/**        P R O G R A M  D I S P L E J A  G R E S A K A             **********/
/***                                                                ***********/
/******************************************************************************/
//

void main(void) {

    GPIO_Init();
    TMR0_Init();
    DSP_10 = LOW;
    DSP_1 = HIGH;
    
    while (1) {

        if (!TMR0) {
            TMR0 = 236;
            mux_input._byte = 0;
            SelectDisplay10Input1();
            __delay_us(5);
            if (!MUX_IN) DSP_10_IN_1 = HIGH;
            SelectDisplay10Input2();
            __delay_us(5);
            if (!MUX_IN) DSP_10_IN_2 = HIGH;
            SelectDisplay10Input4();
            __delay_us(5);
            if (!MUX_IN) DSP_10_IN_4 = HIGH;
            SelectDisplay10Input8();
            __delay_us(5);
            if (!MUX_IN) DSP_10_IN_8 = HIGH;
            SelectDisplay1Input1();
            __delay_us(5);
            if (!MUX_IN) DSP_1_IN_1 = HIGH;
            SelectDisplay1Input2();
            __delay_us(5);
            if (!MUX_IN) DSP_1_IN_2 = HIGH;
            SelectDisplay1Input4();
            __delay_us(5);
            if (!MUX_IN) DSP_1_IN_4 = HIGH;
            SelectDisplay1Input8();
            __delay_us(5);
            if (!MUX_IN) DSP_1_IN_8 = HIGH;
            SH_CLK = LOW;
            SH_DATA = LOW;
            MUX_C = LOW;
            SH_LATCH = LOW;
            shift_data._byte = 0;
            digit = 0;
            if (DSP_10 == HIGH) {
                if (DSP_1_IN_1) digit += 1;
                if (DSP_1_IN_2) digit += 2;
                if (DSP_1_IN_4) digit += 4;
                if (DSP_1_IN_8) digit += 8;
            } else {
                if (DSP_10_IN_1) digit += 1;
                if (DSP_10_IN_2) digit += 2;
                if (DSP_10_IN_4) digit += 4;
                if (DSP_10_IN_8) digit += 8;
            }// End of else...
            if (digit == 0) shift_data._byte = DIGIT_0_MASK;
            else if (digit == 1) shift_data._byte = DIGIT_1_MASK;
            else if (digit == 2) shift_data._byte = DIGIT_2_MASK;
            else if (digit == 3) shift_data._byte = DIGIT_3_MASK;
            else if (digit == 4) shift_data._byte = DIGIT_4_MASK;
            else if (digit == 5) shift_data._byte = DIGIT_5_MASK;
            else if (digit == 6) shift_data._byte = DIGIT_6_MASK;
            else if (digit == 7) shift_data._byte = DIGIT_7_MASK;
            else if (digit == 8) shift_data._byte = DIGIT_8_MASK;
            else if (digit == 9) shift_data._byte = DIGIT_9_MASK;
            else if (digit == 10) shift_data._byte = CHAR_A_MASK;
            else if (digit == 11) shift_data._byte = CHAR_B_MASK;
            else if (digit == 12) shift_data._byte = CHAR_C_MASK;
            else if (digit == 13) shift_data._byte = CHAR_D_MASK;
            else if (digit == 14) shift_data._byte = CHAR_E_MASK;
            else if (digit == 15) shift_data._byte = CHAR_F_MASK;
            if(DSP_10 == HIGH){
                DSP_10 = LOW;
                DSP_1 = HIGH;
            } else {
                DSP_1 = LOW;
            }// End of else...
            if(DSP_1) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_G) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_F) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_E) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_D) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_C) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_B) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            if(SEG_A) SH_DATA = HIGH;
            else SH_DATA = LOW;
            SH_CLK = HIGH;
            SH_CLK = LOW;
            SH_LATCH = HIGH;
            SH_LATCH = LOW;
            SH_DATA = LOW;
            if(DSP_10 == LOW) DSP_10 = HIGH;
        }// End of if...
    }// End of while...
}// End of main...

