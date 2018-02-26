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
	
#define S2	LATBbits.LATB0
#define S3	LATBbits.LATB1
#define S1	LATBbits.LATB2
	
#define D1	PORTBbits.RB4
#define D2	PORTBbits.RB5

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_H */

