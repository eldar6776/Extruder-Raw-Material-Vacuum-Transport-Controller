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
 *  Ime fajla:      delay.h
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
#define PIC_CLK 8000000 // 8Mhz oscilator
//
#ifndef __DELAY_H
#define __DELAY_H
//
extern unsigned char delayus_variable;
//
#if (PIC_CLK == 4000000)
	#define dly1u asm("nop")
	#define dly2u dly1u;dly1u
#elif (PIC_CLK == 8000000)
	#define dly500n asm("nop")
	#define dly1u dly500n;dly500n
	#define dly2u dly1u;dly1u
#elif ( (PIC_CLK == 16000000) || (PIC_CLK == 16257000) )
	#define dly250n asm("nop")
	#define dly500n dly250n;dly250n
	#define dly1u dly500n;dly500n
	#define dly2u dly1u;dly1u
#elif (PIC_CLK == 20000000)
	#define dly200n asm("nop")
	#define dly400n dly250n;dly250n
	#define dly2u dly400n;dly400n;dly400n;dly400n;dly400n
#elif (PIC_CLK == 32000000)
	#define dly125n asm("nop")
	#define dly250n dly125n;dly125n
	#define dly500n dly250n;dly250n
	#define dly1u dly500n;dly500n
	#define dly2u dly1u;dly1u
#else
	#error delay.h - please define pic_clk correctly
#endif
//
//*****
//delay routine
//
#if PIC_CLK == 4000000
	#define DelayDivisor 4
	#define WaitFor1Us asm("nop")
	#define Jumpback asm("goto $ - 2")
#elif PIC_CLK == 8000000
	#define DelayDivisor 2
	#define WaitFor1Us asm("nop")
	#define Jumpback asm("goto $ - 2")
#elif ( (PIC_CLK == 16000000) || (PIC_CLK==16257000) )
	#define DelayDivisor 1
	#define WaitFor1Us asm("nop")
	#define Jumpback asm("goto $ - 2")
#elif PIC_CLK == 20000000
	#define DelayDivisor 1
	#define WaitFor1Us asm("nop"); asm("nop")
	#define Jumpback asm("goto $ - 3")
#elif PIC_CLK == 32000000
	#define DelayDivisor 1
	#define WaitFor1Us asm("nop"); asm("nop"); asm("nop"); asm("nop"); asm("nop")
	#define Jumpback asm("goto $ - 6")
#else
	#error delay.h - please define pic_clk correctly
#endif
//
#define DelayUs(x) { \
			delayus_variable=(unsigned char)(x/DelayDivisor); \
			WaitFor1Us; } \
			asm("decfsz _delayus_variable,f"); \
			Jumpback;
//
#define LOOP_CYCLES_CHAR	9							//how many cycles per loop, optimizations on
#define timeout_char_us(x)	(long)(((x)/LOOP_CYCLES_CHAR)*(PIC_CLK/1000000/4))
//
#define LOOP_CYCLES_INT		16							//how many cycles per loop, optimizations on
#define timeout_int_us(x)	(long)(((x)/LOOP_CYCLES_INT)*(PIC_CLK/1000000/4))
//
//if lo byte is zero, faster initialization by 1 instrucion
#define timeout_int_lobyte_zero_us(x)	(long)(((x)/LOOP_CYCLES_INT)*(PIC_CLK/4.0)&0xFF00)
//
//
/** P R O T O T I P I   F U N K C I J A  *************************************/
//
//
void DelayBigUs(unsigned int cnt);
void DelayMs(unsigned char cnt);
void DelayMs_interrupt(unsigned char cnt);
void DelayBigMs(unsigned int cnt);
void DelayS(unsigned char cnt);
//
//
//
#endif


