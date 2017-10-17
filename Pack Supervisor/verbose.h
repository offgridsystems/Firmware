//---------------------------------------------------------------------------
// Header to handle verbose mode, and cleanup serial output during debuging
// Date       	Who		WAS/IS changes
//---------------------------------------------------------------------------
// 09/14/17		NHJ 	Initial creation

#ifndef VERBOSE_H
#define VERBOSE_H

#ifdef VERBOSE
#include <WProgram.h>

#define VERBOSE_PRINT(x) 		Serial.print(x);
#define VERBOSE_PRINTLN(x)		Serial.println(x);
#define VERBOSE_PRINT_BIN(x)	Serial.print(x, BIN);
#define VERBOSE_PRINTLN_BIN(x)	Serial.println(x, BIN);
#define VERBOSE_PRINT_OCT(x)	Serial.print(x, OCT);
#define VERBOSE_PRINTLN_OCT(x)	Serial.println(x, OCT);
#define VERBOSE_PRINT_DEC(x)	Serial.print(x, DEC);
#define VERBOSE_PRINTLN_DEC(x)	Serial.println(x, DEC);
#define VERBOSE_PRINT_HEX(x)	Serial.print(x, HEX);
#define VERBOSE_PRINTLN_HEX(x)	Serial.println(x, HEX);
#define VERBOSE_PRINT_0(x)		Serial.print(x, 0);
#define VERBOSE_PRINTLN_0(x)	Serial.println(x, 0);
#define VERBOSE_PRINT_1(x)		Serial.print(x, 1);
#define VERBOSE_PRINTLN_1(x)	Serial.println(x, 1);
#define VERBOSE_PRINT_2(x)		Serial.print(x, 2);
#define VERBOSE_PRINTLN_2(x)	Serial.println(x, 2);
#define VERBOSE_PRINT_3(x)		Serial.print(x, 3);
#define VERBOSE_PRINTLN_3(x)	Serial.println(x, 3);
#define VERBOSE_PRINT_4(x)		Serial.print(x, 4);
#define VERBOSE_PRINTLN_4(x)	Serial.println(x, 4);
#define VERBOSE_PRINT_5(x)		Serial.print(x,5);
#define VERBOSE_PRINTLN_5(x)	Serial.println(x,5);
#define VERBOSE_PRINT_6(x)		Serial.print(x,6);
#define VERBOSE_PRINTLN_6(x)	Serial.println(x,6);

#else
#define VERBOSE_PRINT(x)
#define VERBOSE_PRINTLN(x)
#define VERBOSE_PRINT_BIN(x)
#define VERBOSE_PRINTLN_BIN(x)
#define VERBOSE_PRINT_OCT(x)
#define VERBOSE_PRINTLN_OCT(x)
#define VERBOSE_PRINT_DEC(x)
#define VERBOSE_PRINTLN_DEC(x)
#define VERBOSE_PRINT_HEX(x)
#define VERBOSE_PRINTLN_HEX(x)
#define VERBOSE_PRINT_0(x)
#define VERBOSE_PRINTLN_0(x)
#define VERBOSE_PRINT_1(x)
#define VERBOSE_PRINTLN_1(x)
#define VERBOSE_PRINT_2(x)
#define VERBOSE_PRINTLN_2(x)
#define VERBOSE_PRINT_3(x)
#define VERBOSE_PRINTLN_3(x)
#define VERBOSE_PRINT_4(x)
#define VERBOSE_PRINTLN_4(x)
#define VERBOSE_PRINT_5(x)
#define VERBOSE_PRINTLN_5(x)
#define VERBOSE_PRINT_6(x)
#define VERBOSE_PRINTLN_6(x)

#endif //VERBOSE

#ifdef DEBUG
#include <WProgram.h>

#define DEBUG_PRINT(x) 			Serial.print(x);
#define DEBUG_PRINTLN(x)		Serial.println(x);
#define DEBUG_PRINT_BIN(x)		Serial.print(x, BIN);
#define DEBUG_PRINTLN_BIN(x)	Serial.println(x, BIN);
#define DEBUG_PRINT_OCT(x)		Serial.print(x, OCT);
#define DEBUG_PRINTLN_OCT(x)	Serial.println(x, OCT);
#define DEBUG_PRINT_DEC(x)		Serial.print(x, DEC);
#define DEBUG_PRINTLN_DEC(x)	Serial.println(x, DEC);
#define DEBUG_PRINT_HEX(x)		Serial.print(x, HEX);
#define DEBUG_PRINTLN_HEX(x)	Serial.println(x, HEX);
#define DEBUG_PRINT_0(x)		Serial.print(x, 0);
#define DEBUG_PRINTLN_0(x)		Serial.println(x, 0);
#define DEBUG_PRINT_1(x)		Serial.print(x, 1);
#define DEBUG_PRINTLN_1(x)		Serial.println(x, 1);
#define DEBUG_PRINT_2(x)		Serial.print(x, 2);
#define DEBUG_PRINTLN_2(x)		Serial.println(x, 2);
#define DEBUG_PRINT_3(x)		Serial.print(x, 3);
#define DEBUG_PRINTLN_3(x)		Serial.println(x, 3);
#define DEBUG_PRINT_4(x)		Serial.print(x, 4);
#define DEBUG_PRINTLN_4(x)		Serial.println(x, 4);
#define DEBUG_PRINT_5(x)		Serial.print(x,5);
#define DEBUG_PRINTLN_5(x)		Serial.println(x,5);
#define DEBUG_PRINT_6(x)		Serial.print(x,6);
#define DEBUG_PRINTLN_6(x)		Serial.println(x,6);

#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINT_BIN(x)
#define DEBUG_PRINTLN_BIN(x)
#define DEBUG_PRINT_OCT(x)
#define DEBUG_PRINTLN_OCT(x)
#define DEBUG_PRINT_DEC(x)
#define DEBUG_PRINTLN_DEC(x)
#define DEBUG_PRINT_HEX(x)
#define DEBUG_PRINTLN_HEX(x)
#define DEBUG_PRINT_0(x)
#define DEBUG_PRINTLN_0(x)
#define DEBUG_PRINT_1(x)
#define DEBUG_PRINTLN_1(x)
#define DEBUG_PRINT_2(x)
#define DEBUG_PRINTLN_2(x)
#define DEBUG_PRINT_3(x)
#define DEBUG_PRINTLN_3(x)
#define DEBUG_PRINT_4(x)
#define DEBUG_PRINTLN_4(x)
#define DEBUG_PRINT_5(x)
#define DEBUG_PRINTLN_5(x)
#define DEBUG_PRINT_6(x)
#define DEBUG_PRINTLN_6(x)

#endif //DEBUG

#endif //VERBOSE.h