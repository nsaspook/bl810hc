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

#define MAX_ADC_CHAN	5

	typedef enum {
		APP_STATE_INIT = 0,
		APP_STATE_WAIT_INPUT,
		APP_STATE_COMMAND,
		APP_STATE_EXECUTE,
		APP_STATE_TEST,
		APP_STATE_4,
		APP_STATE_5,
		/* Application Error state*/
		APP_STATE_ERROR
	} APP_STATES;

	typedef enum {
		ADC_FBACK = 0,
		ADC_AUX,
		ADC_CW,
		ADC_CCW,
		ADC_SPARE
	} ADC_STATES;

	typedef enum {
		CMD_IDLE = 0,
		CMD_WAIT,
		CMD_CW,
		CMD_CCW,
		CMD_ON,
		CMD_OFF
	} CMD_STATES;

	typedef struct V_data { // control data structure with possible volatile issues
		volatile uint8_t b_data, adc_i, blink, onled, db1, db2, odelay, bdelay;
		volatile uint8_t adc_flag : 1;
		volatile uint8_t run : 1;
		volatile int8_t runcount;
		volatile uint8_t cw : 1;
		volatile uint8_t ccw : 1;
		volatile uint8_t reset : 1;
		volatile uint8_t button1 : 1;
		volatile uint8_t button2 : 1;
		volatile uint16_t adc_data[MAX_ADC_CHAN];
		volatile uint32_t sequence, sequence_save;
		uint8_t str[64];
		volatile APP_STATES motor_state;
		volatile ADC_STATES adc_state;
		volatile CMD_STATES cmd_state;
	} V_data;

	typedef struct pottype {
		int16_t pos_actual, pos_set, error, pos_actual_prev, pos_change; // in ADC counts
		int16_t limit_change, limit_span, limit_offset, limit_offset_l, limit_offset_h; // AXIS limits for error checking
		int16_t low, high, offset, span, cal_low, cal_high, cal_failed, cal_warn; // end of travel ADC count values
		float scale_out, scale_in; // scaling factor from actual to scaled and back
		int16_t scaled_actual, scaled_set, scaled_error; // 0..1023 value of pot for LCD readback
	} volatile pottype;

	typedef struct motortype {
		uint8_t type, run, cw, axis, free, slow, active, reversed, v24, slow_only, on_off_only;
		int16_t hunt_count, cal_pos;
		struct pottype pot;
	} volatile motortype;

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

#define ELED1	LATEbits.LATE6
#define ELED2	LATEbits.LATE7
#define LEDS            LATE

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_H */

