/******************************************************************************
 *
 *           Kontroler vakum transporta sirovine linije 5
 *
 *                         Definicije tipova
 *
 *                       Firmware versija 1.02
 *
 ******************************************************************************
 *
 *  Ime fajla:      typedefs.h
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
#ifndef TYPEDEFS_H
#define TYPEDEFS_H
//
//
//
typedef unsigned char byte; // 8-bit
typedef unsigned int word; // 16-bit
typedef unsigned long dword; // 32-bit
//
typedef union _BYTE {//start of typedefs union _BYTE
    byte _byte;

    struct {//start of struct
        unsigned b0 : 1;
        unsigned b1 : 1;
        unsigned b2 : 1;
        unsigned b3 : 1;
        unsigned b4 : 1;
        unsigned b5 : 1;
        unsigned b6 : 1;
        unsigned b7 : 1;
    };//end of struct
} BYTE;//end of typedefs union _BYTE

typedef union _WORD {//start of typedef union _WORD
    word _word;

    struct {//start of struct
        byte byte0;
        byte byte1;
    };//end of struct

    struct {//start of struct
        BYTE Byte0;
        BYTE Byte1;
    };//end of struct

    struct {//start of struct
        BYTE LowB;
        BYTE HighB;
    };//end of struct

    struct {//start of struct
        byte v[2];
    };//end of struct
} WORD;//end of typedefs _WORD
#define LSB(a)      ((a).v[0])
#define MSB(a)      ((a).v[1])

typedef union _DWORD {//start of typedefs union _DWORD
    dword _dword;

    struct {//start of struct
        byte byte0;
        byte byte1;
        byte byte2;
        byte byte3;
    };//end of struct

    struct {//start of struc
        word word0;
        word word1;
    };//end of struct

    struct {//start of struct
        BYTE Byte0;
        BYTE Byte1;
        BYTE Byte2;
        BYTE Byte3;
    };//end of struc

    struct {//start of struct
        WORD Word0;
        WORD Word1;
    };//end of sturct

    struct {//start of struct
        byte v[4];
    };//end of struct
} DWORD;//end of typedefs _DWORD

#define LOWER_LSB(a)    ((a).v[0])
#define LOWER_MSB(a)    ((a).v[1])
#define UPPER_LSB(a)    ((a).v[2])
#define UPPER_MSB(a)    ((a).v[3])

typedef enum _BOOL {
    FALSE = 0, TRUE
} BOOL;


/**  D E F I N I C I J E ******************************************************/
//......................................opste definicije
#define	TRUE 1
#define	FALSE 0
#define HIGH 1
#define LOW 0
#define OUT 0
#define IN 1
#define INPUT_PIN	1
#define OUTPUT_PIN	0
#define OK      TRUE
#define FAIL    FALSE

#endif //TYPEDEFS_H
