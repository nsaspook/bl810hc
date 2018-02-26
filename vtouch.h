/* 
 * File:   vtouch.h
 * Author: root
 *
 * Created on September 20, 2017, 12:27 PM
 */

#ifndef VTOUCH_H
#define	VTOUCH_H

#ifdef	__cplusplus
extern "C" {
#endif

#ifdef INTTYPES
#include <stdint.h>
#else
#define INTTYPES
	/*unsigned types*/
	typedef unsigned char uint8_t;
	typedef unsigned int uint16_t;
	typedef unsigned long uint32_t;
	typedef unsigned long long uint64_t;
	/*signed types*/
	typedef signed char int8_t;
	typedef signed int int16_t;
	typedef signed long int32_t;
	typedef signed long long int64_t;
#endif	

#define	TIMEROFFSET	26474			// timer0 16bit counter value for 1 second to overflow
#define	TIMERFAST	58974			// fast flash or testing
	
#define S2	LATBbits.LATB0
#define S3	LATBbits.LATB1
#define S1	LATBbits.LATB2
	
#define D1	PORTBbits.RB4
#define D2	PORTBbits.RB5

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_H */

