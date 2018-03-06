
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
		ADC_POT
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
		volatile uint8_t b_data, adc_i, blink, onled, db1, db2, odelay, bdelay, sequence;
		volatile uint8_t buzzertime; // 20Hz timer counter event registers
		volatile bool adc_flag;
		volatile bool run, testing, stable;
		volatile int8_t runcount;
		volatile bool cw;
		volatile bool ccw;
		volatile bool reset;
		volatile bool button1;
		volatile bool button2;
		volatile bool stopped;
		volatile uint16_t adc_data[MAX_ADC_CHAN];
		volatile uint32_t clock20;
		uint8_t str[64];
		volatile APP_STATES motor_state;
		volatile ADC_STATES adc_state;
		volatile CMD_STATES cmd_state;
	} V_data;

	enum movement_t {
		CW, STOP_M, CCW
	} movement_t;

	typedef struct pottype {
		enum movement_t movement;
		int16_t pos_actual, pos_set, error, pos_actual_prev, pos_change; // in ADC counts
		int16_t limit_change, limit_span, limit_offset, limit_offset_l, limit_offset_h; // AXIS limits for error checking
		int16_t low, high, offset, span, cal_low, cal_high, cal_warn; // end of travel ADC count values
		float scale_out, scale_in; // scaling factor from actual to scaled and back
		int16_t scaled_actual, scaled_set, scaled_error;
		bool cal_failed;
		uint8_t cw, ccw;
	} volatile pottype;

	typedef struct motortype {
		uint8_t type, run, cw, axis, free, slow, active, reversed, v24, slow_only, on_off_only;
		int16_t hunt_count, cal_pos;
		struct pottype pot;
	} volatile motortype;

	typedef struct R_data { // set only in adc_read
		uint32_t systemvoltage, motorvoltage, pos_x, change_x, max_x;
		int32_t current_x;
		bool stable_x;
	} R_data;

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

#define POT_MAX_CHANGE  500             // if the change in readback between ADC reads is this or greater, it's a possible error
#define POT_M_OFFSET	500		// offset mean
#define POT_H_OFFSET	999             // offset high fail limit
#define POT_L_OFFSET	0               // offset low fail limit
#define POT_MIN_SPAN    200             // if the change in readback between ADC reads is this or less, it's a possible error
#define CHANGE_COUNT    20            	// number of ADC updates before the R.change_ variable are updated
#define	MIN_CHANGE	10l					// ADC counts change between stable checks

#define SCALED          999
#define SCALED_FLOAT    999.9
#define ACTUAL          4095			// adc counts

#ifdef	__cplusplus
}
#endif

#endif	/* VTOUCH_H */

