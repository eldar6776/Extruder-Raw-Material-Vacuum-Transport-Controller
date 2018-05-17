/******************************************************************************
 *
 *   Kontroler transporta materijala na mikser sirovine linije 5
 *
 *                          modul pauze
 *
 *******************************************************************************
 * Ime fajla:		delay.c
 *
 * Procesor:        	PIC16F877
 *
 * Kompajler:       	Microchip XC8 v1.31
 *
 * Datum:		maj 2014.
 *
 * mailto               <eldar6776@hotmail.com>
 *
 ******************************************************************************/
//
//
//
#ifndef __DELAY_C
#define __DELAY_C

#include <pic.h>
#include "io_cfg.h"

unsigned char delayus_variable;

#include	"delay.h"

void DelayBigUs(unsigned int cnt) {
    unsigned char i;

    i = (unsigned char) (cnt >> 8);
    while (i >= 1) {
        i--;
        DelayUs(253);
        CLRWDT();
    }
    DelayUs((unsigned char) (cnt & 0xFF));
}

void DelayMs(unsigned char cnt) {
    unsigned char i;
    do {
        i = 4;
        do {
            DelayUs(250);
            CLRWDT();
        } while (--i);
    } while (--cnt);
}

//this copy is for the interrupt function

void DelayMs_interrupt(unsigned char cnt) {
    unsigned char i;
    do {
        i = 4;
        do {
            DelayUs(250);
        } while (--i);
    } while (--cnt);
}

void DelayBigMs(unsigned int cnt) {
    unsigned char i;
    do {
        i = 4;
        do {
            DelayUs(250);
            CLRWDT();
        } while (--i);
    } while (--cnt);
}

void DelayS(unsigned char cnt) {
    unsigned char i;
    do {
        i = 4;
        do {
            DelayMs(250);
            CLRWDT();
        } while (--i);
    } while (--cnt);
}

#endif


