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

#define MAX_ADC	5

	typedef struct V_data { // control data structure with possible volatile issues
		volatile uint8_t b_data, adc_i;
		volatile uint8_t adc_flag : 1;
		volatile uint16_t adc_data[MAX_ADC];
		uint8_t str[64];
	} V_data;

	typedef enum {
		ADC_FBACK = 0,
		ADC_AUX,
		ADC_CW,
		ADC_CCW,
		ADC_SPARE
	} ADC_STATES;

#define S2	LATBbits.LATB0
#define S3	LATBbits.LATB1
#define S1	LATBbits.LATB2

#define D1	PORTBbits.RB4
#define D2	PORTBbits.RB5

#define OPTO1	PORTEbits.RE0
#define OPTO2	PORTEbits.RE1

#define BUTTON1	PORTEbits.RE2
#define BUTTON2	PORTEbits.RE3

#define BLED1	LATEbits.LATE4
#define BLED2	LATEbits.LATE5

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_H */

