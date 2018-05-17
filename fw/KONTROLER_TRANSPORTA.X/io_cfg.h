/******************************************************************************
 *
 *           Kontroler vakum transporta sirovine linije 5
 *
 *                        Definicije hardvera
 *
 *                       Firmware verzija 1.02
 *
 ******************************************************************************
 *
 *  Ime fajla:      io_cfg.h
 *
 *  Procesor:       PIC16F1517
 *
 *  Kompajler:      Microchip XC8 v1,13
 *
 *  IDE:            MPLAB X IDE v2.10 (java 1.7.0_25)
 *
 *  Datum:          20. jun 2014.
 *
 *  Autor:          eldar6776@hotmail.com
 *
 ******************************************************************************/
//
//
//
#ifndef IO_CFG_H
#define IO_CFG_H
//
//
/******** D E F I N I C I J E    P O R T O V A *******************************/
//
//
//---------------------------------------------------- PORTA
//
#define ANIN_VP1_VRIJEME_ZADRSKE        RA0
#define ANIN_VP1_BRZINA_PUNJENJA        RA1
#define ANIN_VP1_REDUKOVANA_BRZINA      RA2
#define ANIN_MAX_VRIJEME_PUNJENJA       RA3
#define DIN_DRIVE_OPTION                RA4
#define AININ_MILL_SHUTDOWN_DELAY       RA5
#define InitPORTA()                     (LATA = 0x00, TRISA = 0xff, ANSELA = 0xff)
//
//---------------------------------------------------- PORTB
//
#define AUX_INT0                        RB0
#define AUX_INT0_DIR                    TRISB0
#define DOUT_SIG1_CLK                   RB1
#define DOUT_SIG2_CLK                   RB2
#define DOUT_DAC1_LATCH                 RB3
#define DOUT_DAC1_CS                    RB4
#define DOUT_SHIFT_OE                   RB5
#define ICSP_PGC                        RB6
#define ICSP_PGD                        RB7
#define InitPORTB()                     (LATB = 0x00, TRISB = 0x00, ANSELB = 0x00)
//
//---------------------------------------------------- PORTC
//
#define RS485_DATA_DIR                  RC0
#define PWM1_OUT                        RC1
#define PWM2_OUT                        RC2
#define DOUT_SERIAL_CLOCK               RC3
#define DOUT_SHIFT_LATCH                RC4
#define DOUT_SERIAL_DATA                RC5
#define RS485_DATA_TX                   RC6
#define RS485_DATA_RX                   RC7
#define InitPORTC()                     (LATC = 0x00, TRISC = 0x80, ANSELC = 0x00)
//
//---------------------------------------------------- PORTD
//
#define DATA0                           RD0
#define DATA1                           RD1
#define DATA2                           RD2
#define DATA3                           RD3
#define DATA4                           RD4
#define DATA5                           RD5
#define DATA6                           RD6
#define DATA7                           RD7
#define DataWrite()                     PORTD
#define DataRead()                      PORTD
#define DataDir()                       TRISD
#define InitPORTD()                     (PORTD = 0x00, TRISD = 0xff, ANSELD = 0x00)
//
//---------------------------------------------------- PORTE
//
#define ANIN_FEEDER4_VRIJEME_PUNJENJA   RE0
#define ANIN_VP1_CLEANING_CYCLES        RE1
#define DIN_EXTRUDER_RUN                RE2
#define InitPORTE()                     (LATE = 0x00, TRISE = 0xff, ANSELE = 0x03)
//
//
//
#endif	//	IO_CFG_H