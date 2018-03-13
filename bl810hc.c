
// PIC18F8723 Configuration Bit Settings

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF         // Watchdog Timer (WDT off)
#pragma config WDTPS = 128      // Watchdog Timer Postscale Select bits (1:128)

// CONFIG3L
#pragma config MODE = MC        // Processor Data Memory Mode Select bits (Microcontroller mode)
#pragma config ADDRBW = ADDR20BIT// Address Bus Width Select bits (20-bit Address Bus)
#pragma config DATABW = DATA16BIT// Data Bus Width Select bit (16-bit External Bus mode)
#pragma config WAIT = OFF       // External Bus Data Wait Enable bit (Wait selections are unavailable for table reads and table writes)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (ECCP2 input/output is multiplexed with RC1)
#pragma config ECCPMX = PORTE   // ECCP MUX bit (ECCP1/3 (P1B/P1C/P3B/P3C) are multiplexed onto RE6, RE5, RE4 and RE3 respectively)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RG5 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config BBSIZ = BB2K     // Boot Block Size Select bits (1K word (2 Kbytes) Boot Block size)
#pragma config XINST = OFF   // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit Block 3 (Block 3 (00C000-00FFFFh) not code-protected)
#pragma config CP4 = OFF        // Code Protection bit Block 4 (Block 4 (010000-013FFFh) not code-protected)
#pragma config CP5 = OFF        // Code Protection bit Block 5 (Block 5 (014000-017FFFh) not code-protected)
#pragma config CP6 = OFF        // Code Protection bit Block 6 (Block 6 (01BFFF-018000h) not code-protected)
#pragma config CP7 = OFF        // Code Protection bit Block 7 (Block 7 (01C000-01FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot Block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit Block 3 (Block 3 (00C000-00FFFFh) not write-protected)
#pragma config WRT4 = OFF       // Write Protection bit Block 4 (Block 4 (010000-013FFFh) not write-protected)
#pragma config WRT5 = OFF       // Write Protection bit Block 5 (Block 5 (014000-017FFFh) not write-protected)
#pragma config WRT6 = OFF       // Write Protection bit Block 6 (Block 6 (01BFFF-018000h) not write-protected)
#pragma config WRT7 = OFF       // Write Protection bit Block 7 (Block 7 (01C000-01FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit Block 0 (Block 0 (000800, 001000 or 002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR4 = OFF      // Table Read Protection bit Block 4 (Block 4 (010000-013FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR5 = OFF      // Table Read Protection bit Block 5 (Block 5 (014000-017FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR6 = OFF      // Table Read Protection bit Block 6 (Block 6 (018000-01BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR7 = OFF      // Table Read Protection bit Block 7 (Block 7 (01C000-01FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-007FFF, 000FFF or 001FFFh) not protected from table reads executed in other blocks)



#include <xc.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "bl810hc.h"
#include "bl810hc_build.h"
#include "vector.h"

void ADC_read(void);

volatile uint8_t sequence = 0;
struct V_data V;
struct R_data R;

volatile struct motortype motordata[1], *motor_ptr; // use array for possible dual motor

static uint8_t bootstr2[128];
static uint32_t rawp[1], rawa[1]; // use 32-bits for possible average function

extern const uint16_t TIMEROFFSET, TIMERDEF, TIMER3REG;
extern const uint8_t TIMER4DEF;

void w_time(uint32_t delay) // delay = ~ .01 seconds
{
	uint32_t clocks_hz;

	clocks_hz = clock10() + delay;
	do { // wait until delay
		Nop();
	} while (clock10() < clocks_hz);
}

void USART_putc(uint8_t c)
{
	while (!TXSTA1bits.TRMT);
	TXREG1 = c;
}

void USART_puts(uint8_t *s)
{
	while (*s) {
		USART_putc(*s);
		s++;
	}
}

void USART_putsr(const uint8_t *s)
{
	while (*s) {
		USART_putc(*s);
		s++;
	}
}

void term_time(void)
{
	bootstr2[0] = 0;
}

void putrs2USART(const uint8_t *s)
{
	while (*s) {
		USART_putc(*s);
		s++;
	}
}

void puts2USART(uint8_t *s)
{
	while (*s) {
		USART_putc(*s);
		s++;
	}
}

int16_t ABSI(int16_t i)
{
	if (i < 0)
		return -i;
	else
		return i;
}

int32_t ABSL(int32_t i)
{
	if (i < 0)
		return -i;
	else
		return i;
}

bool Change_Count(void)
{
	if (V.change_count++ >= CHANGE_COUNT) {
		if ((ABSL(R.pos_x - R.change_x) < MIN_CHANGE))
			R.stable_x = true;
		V.change_count = CHANGE_COUNT;
		return true;
	}
	return false; // wait for CHANGE_COUNT times
}

void Reset_Change_Count(void)
{
	V.change_count = 0;
	R.change_x = R.pos_x;
	R.stable_x = false;
}

void init_motor(void)
{
	V.stable = false;
	V.change_count = 0;
	ADC_read();
	motordata[0].run = true;
	motordata[0].cw = true;
	motordata[0].axis = 0;
	motordata[0].hunt_count = 0;
	motordata[0].free = true;
	motordata[0].slow = false;
	motordata[0].slow_only = false;
	motordata[0].active = true;
	motordata[0].reversed = false;
	motordata[0].v24 = false;
	motordata[0].cal_pos = 500;
	motordata[0].pot.pos_set = 2047; // mid range
	motordata[0].pot.pos_actual = rawp[0]; // set to current pot reading
	motordata[0].pot.scaled_set = 500; // mid range
	motordata[0].pot.pos_actual_prev = rawp[0];
	motordata[0].pot.movement = STOP_M; // no motion
	motordata[0].pot.pos_change = 0; // pot position change mag
	motordata[0].pot.low = 4095; // init limit detection values
	motordata[0].pot.high = 0; // ditto
	motordata[0].pot.cal_low = false;
	motordata[0].pot.cal_high = false;
	motordata[0].pot.cal_failed = false;
	motordata[0].pot.cal_warn = false;
	motordata[0].pot.limit_change = POT_MAX_CHANGE;
	motordata[0].pot.limit_span = POT_MIN_SPAN;
	motordata[0].pot.limit_offset = POT_M_OFFSET;
	motordata[0].pot.limit_offset_l = POT_L_OFFSET;
	motordata[0].pot.limit_offset_h = POT_H_OFFSET;
}

void ADC_read(void) // update all voltage/current readings and set load current in 'currentload' variable
{ // ADC is opened and config'd in main
	uint8_t z; // used for fast and slow sample loops >256

	di();
	rawp[0] = V.adc_data[ADC_FBACK];
	rawa[0] = V.adc_data[ADC_AUX];
	ei();
	R.pos_x = rawp[0];
	R.max_x = rawa[0];
	z = 0;
	motordata[0].pot.pos_actual = rawp[0];
	if (ABSI(motordata[0].pot.pos_actual - motordata[0].pot.pos_actual_prev) > motordata[0].pot.pos_change) {
		motordata[0].pot.pos_change = ABSI(motordata[0].pot.pos_actual - motordata[0].pot.pos_actual_prev);
	}
	if (motordata[0].pot.pos_change > motordata[0].pot.limit_change && V.stable) {
		term_time();
		sprintf(bootstr2, "\r\n Pot %i Change too high %i\r\n", z, motordata[0].pot.pos_change);
		puts2USART(bootstr2);
		motordata[0].pot.pos_change = motordata[0].pot.limit_change; // after one message stop and set it to the limit.
	}
	// Check for POT Dead-Spot readings
	if (motordata[0].pot.pos_change >= POT_MAX_CHANGE && V.stable)
		motordata[0].pot.cal_failed = true;

	// update the travel limits and values
	motordata[0].pot.pos_actual_prev = motordata[0].pot.pos_actual;
	if (motordata[0].pot.pos_actual > motordata[0].pot.high)
		motordata[0].pot.high = motordata[0].pot.pos_actual;
	if (motordata[0].pot.pos_actual < motordata[0].pot.low)
		motordata[0].pot.low = motordata[0].pot.pos_actual;
	motordata[0].pot.offset = motordata[0].pot.low;
	motordata[0].pot.span = motordata[0].pot.high - motordata[0].pot.low;
	if (motordata[0].pot.span < 0)
		motordata[0].pot.span = 0;
	motordata[0].pot.scale_out = SCALED_FLOAT / motordata[0].pot.span;
	motordata[0].pot.scale_in = motordata[0].pot.span / SCALED_FLOAT;
	motordata[0].pot.scaled_actual = (int16_t) ((float) (motordata[0].pot.pos_actual - motordata[0].pot.offset) * motordata[0].pot.scale_out);
	if (motordata[0].pot.scaled_actual > SCALED)
		motordata[0].pot.scaled_actual = SCALED;
	motordata[0].pot.pos_set = (int16_t) (((float) motordata[0].pot.scaled_set * motordata[0].pot.scale_in) + motordata[0].pot.offset);
}

void display_cal(void)
{
	ADC_read();

	sprintf(bootstr2, "Position ADC:  %li ", R.pos_x);
	puts2USART(bootstr2);
	sprintf(bootstr2, " Pot CHANGE: %i ", motordata[0].pot.pos_change);
	puts2USART(bootstr2);
	sprintf(bootstr2, " Pot LOW: %i ", motordata[0].pot.low);
	puts2USART(bootstr2);
	sprintf(bootstr2, " Pot HIGH: %i ", motordata[0].pot.high);
	puts2USART(bootstr2);
	sprintf(bootstr2, "Pot VALUE: %i ", motordata[0].pot.pos_actual);
	puts2USART(bootstr2);
	sprintf(bootstr2, "Pot MAX VALUE: %li ", R.max_x);
	puts2USART(bootstr2);
	sprintf(bootstr2, "Position: %i\r\n", motordata[0].pot.scaled_actual);
	puts2USART(bootstr2);
}

uint8_t checktime_cal(uint32_t delay, uint8_t set) // delay = ~ .05 seconds
{
	static uint32_t dcount, timetemp, clocks_hz;

	if (set) {
		di();
		dcount = V.clock10;
		ei();
		clocks_hz = dcount + delay;
	}

	di();
	timetemp = V.clock10;
	ei();
	if (timetemp < clocks_hz) return false;
	return true;
}

/* assembly calibration and test routines */
void run_cal(void) // routines to test and set position data for assy motors or valves
{
	uint32_t z, motor_counts = 1000;
	int8_t p = 'X';

	term_time();
	putrs2USART("\x1b[7m Calibrate/Test Assy(s). \x1b[0m\r\n");
	z = 0;

	checktime_cal(motor_counts, true);
	Reset_Change_Count();
	V.stopped = false;
	run_ccw();

	ADC_read();
	ADC_read();
	motordata[0].pot.cal_failed = false;

	/* normal motor tests */
	do {
		if (z % 1000 == 0) {
			V.stable = true; // start qualifying the motor now
			term_time();
			puts2USART(bootstr2);
			sprintf(bootstr2, "Calibrate CCW %lu ", z); // info display data
			puts2USART(bootstr2);
			display_cal();
			if (Change_Count()) {
				if (R.stable_x) {
					term_time();
					sprintf(bootstr2, " NO ADC VOLTAGE CHANGE DETECTED \r\n\r\n");
					puts2USART(bootstr2);
					V.stopped = true;
				}
				Reset_Change_Count();
			}
		}
		if (V.opto2) {// stop at end of travel flag
			V.stopped = true;
			sprintf(bootstr2, " At Limit\r\n");
			puts2USART(bootstr2);
		}
		z++;
	} while (!checktime_cal(motor_counts, false)&& !V.stopped);

	sprintf(bootstr2, " Forward motion done \r\n");
	puts2USART(bootstr2);
	w_time(20);
	sprintf(bootstr2, " Reverse motion start\r\n");
	puts2USART(bootstr2);

	z = 0;
	checktime_cal(motor_counts, true);
	Reset_Change_Count();
	V.stopped = false;
	run_cw();

	ADC_read();
	ADC_read();
	do {
		if (z % 1000 == 0) {
			term_time();
			puts2USART(bootstr2);
			sprintf(bootstr2, "Calibrate CW %lu  ", z); // info display data
			puts2USART(bootstr2);
			display_cal();
			if (Change_Count()) {
				if (R.stable_x) {
					term_time();
					sprintf(bootstr2, " NO ADC VOLTAGE CHANGE DETECTED \r\n\r\n");
					puts2USART(bootstr2);
					V.stopped = true;
				}
				Reset_Change_Count();
			}
		}
		if (V.opto1) {
			V.stopped = true;
			sprintf(bootstr2, " At Limit\r\n");
			puts2USART(bootstr2);
		}
		z++;
	} while (!checktime_cal(motor_counts, false) && !V.stopped);
	run_stop();

	if ((motordata[0].pot.pos_change > motordata[0].pot.limit_change))
		motordata[0].pot.cal_failed = true;
	if ((motordata[0].pot.span < motordata[0].pot.limit_span))
		motordata[0].pot.cal_failed = true;
	if (motordata[0].pot.offset > motordata[0].pot.limit_offset_h)
		motordata[0].pot.cal_failed = true;
	if (motordata[0].pot.offset < motordata[0].pot.limit_offset_l)
		motordata[0].pot.cal_warn = true;

	if (!motordata[0].pot.cal_failed) {
		motordata[0].pot.cal_low = true;
		motordata[0].pot.cal_high = true;
		motordata[0].pot.scaled_set = motordata[0].cal_pos; // move to install position
		p = 'A';
		term_time();
		if (!motordata[0].pot.cal_warn) {
			sprintf(bootstr2, "\x1b[7m Calibrate/Test motor %c PASSED. \x1b[0m\r\n", p);
		} else {
			sprintf(bootstr2, "\x1b[7m Calibrate/Test motor %c PASSED with WARNING. \x1b[0m\r\n", p);
		}
		puts2USART(bootstr2);
		sprintf(bootstr2, " If Dead   %i < %i      ", motordata[0].pot.pos_change, motordata[0].pot.limit_change);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, " If Span   %i > %i      ", motordata[0].pot.span, motordata[0].pot.limit_span);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, " If Offset %i <%i >%i    ", motordata[0].pot.offset, motordata[0].pot.limit_offset_h, motordata[0].pot.limit_offset_l);
		puts2USART(bootstr2);
		putrs2USART("\r\n");

		sprintf(bootstr2, "\r\n Move to Center Position \r\n");
		puts2USART(bootstr2);
		Reset_Change_Count();
		V.stopped = false;
		run_ccw();
		do {
			ADC_read();
			if (motordata[0].pot.scaled_actual > 500)
				V.stopped = true;
			if (V.opto2) {// stop at end of travel flag
				V.stopped = true;
				sprintf(bootstr2, " At Limit\r\n");
				puts2USART(bootstr2);
			}
		} while (!checktime_cal(motor_counts, false)&& !V.stopped);
		run_stop();
		display_cal();
	} else {
		p = 'A';
		term_time();
		putrs2USART(" ");
		sprintf(bootstr2, "Motor %c FAILED cal  ", p); // info display data
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, " If Dead   %i > %i      ", motordata[0].pot.pos_change, motordata[0].pot.limit_change);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, " If Span   %i < %i      ", motordata[0].pot.span, motordata[0].pot.limit_span);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		sprintf(bootstr2, " If Offset %i >%i <%i    ", motordata[0].pot.offset, motordata[0].pot.limit_offset_h, motordata[0].pot.limit_offset_l);
		puts2USART(bootstr2);
		putrs2USART("\r\n");
		p = 'A';
		term_time();
		sprintf(bootstr2, "\x1b[7m Calibrate/Test motor %c FAILED. \x1b[0m\r\n", p);
		puts2USART(bootstr2);
	}

	term_time();
	putrs2USART("\x1b[7m Calibrate/Test Completed. \x1b[0m\r\n");
}

void init_cpu_hw(void)
{
	INTCON = 0;
	INTCON3bits.INT1IE = 0;
	INTCON3bits.INT2IE = 0;
	INTCON3bits.INT3IE = 0;

	// default interface
	// port A to default
	PIE1bits.ADIE = 1;
	ADCON0bits.ADON = 1;
	ADCON0bits.CHS = 0;
	ADCON1bits.VCFG = 0;
	ADCON1bits.PCFG = 0b1010;
	ADCON2bits.ACQT = 0b111;
	ADCON2bits.ADCS = 0b110;
	ADCON2bits.ADFM = 1;

	TRISG = 0;
	LATG = 0;
	LATGbits.LATG3 = 1;
	LATGbits.LATG4 = 1;

	LATB = 0xff;
	TRISB = 0b00110000; // read LEDs and set switches

	TRISC = 0;
	LATC = 0;
	TRISD = 0; // DIAG signal port
	TRISE = 0b00001111;
	LATE = 0x0F;
	TRISF = 0;
	LATF = 0xFF;
	TRISH = 0;
	LATH = 0;
	TRISJ = 0;

	/*
	 * Open the USART configured as
	 * 8N1, 9600 baud, in receive INT mode
	 */
	TXSTA1bits.TXEN = 1;
	RCSTA1bits.CREN = 1;
	RCSTA1bits.SPEN = 1;
	TXSTA1bits.SYNC = 0;
	TXSTA1bits.SYNC = 0;
	TXSTA1bits.BRGH = 0;
	BAUDCON1bits.BRG16 = 0;
	SPBRGH1 = 0;
	SPBRG1 = 64; /* 9600 baud */
	PIE1bits.RC1IE = 1; // enable rs232 serial receive interrupts

	TXSTA2bits.TXEN = 1;
	RCSTA2bits.CREN = 1;
	RCSTA2bits.SPEN = 1;
	TXSTA2bits.SYNC = 0;
	TXSTA2bits.SYNC = 0;
	TXSTA2bits.BRGH = 0;
	BAUDCON2bits.BRG16 = 0;
	SPBRGH2 = 0;
	SPBRG2 = 64; /* 9600 baud */
	PIE3bits.RC2IE = 1;

	//	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
	T0CON = 0b10000111;
	WRITETIMER0(TIMEROFFSET);
	INTCONbits.TMR0IE = 1; // enable int
	INTCON2bits.TMR0IP = 1;

	//	OpenTimer1(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_2 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF); // strobe position clock
	T1CON = 0b10010101;
	T1CONbits.TMR1ON = 1;
	WRITETIMER1(TIMERDEF);
	PIE1bits.TMR1IE = 1;
	IPR1bits.TMR1IP = 1;

	//		OpenTimer3(TIMER_INT_ON & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_8 & T1_OSC1EN_OFF & T1_SYNC_EXT_OFF);
	T3CON = 0b10011101;
	T3CONbits.TMR3ON = 1;
	WRITETIMER3(TIMER3REG);
	IPR2bits.TMR3IP = 1;
	PIE2bits.TMR3IE = 1;

	// OpenTimer4
	T4CON = 0b01111111;
	PR4 = TIMER4DEF;
	PIE3bits.TMR4IE = 1;

	/* Display a prompt to the USART */
	USART_putsr(build_version);
	USART_putsr(" ");
	USART_putsr(build_date);
	USART_putsr(" ");
	USART_putsr(build_time);
	USART_putsr(" ");

	PIR1bits.RC1IF = 0;
	PIR3bits.RC2IF = 0;
	PIR1bits.TX1IF = 0;
	PIR3bits.TX2IF = 0;
	INTCONbits.PEIE = 1;
	INTCONbits.GIEH = 1; // enable high ints	
}

void main(void)
{
	uint8_t tester[] = "\r\n 810HC Brushless motor tester ";
	V.stable = false;
	V.adc_i = 0;
	V.motor_state = APP_STATE_INIT;
	V.adc_state = ADC_FBACK;
	V.cmd_state = CMD_IDLE;
	V.sequence = 0;


	init_cpu_hw();
	USART_puts(tester);
	init_motor();

	/* Loop forever */
	while (true) {
		switch (V.motor_state) {
		case APP_STATE_INIT:
			V.adc_i = 0;
			V.adc_state = ADC_FBACK;
			V.sequence = 0;
			V.motor_state = APP_STATE_WAIT_INPUT;
			V.cmd_state = CMD_IDLE;
			BLED1 = false;
			BLED2 = false;
			ELED1 = false;
			ELED2 = false;
			break;
		case APP_STATE_WAIT_INPUT:
			switch (V.sequence) {
			case 1:
				S1 = 0;
				break;
			case 2:
				S1 = 1;
				break;
			case 3:
				USART_putsr("\r\n ");
				utoa(V.str, V.adc_data[ADC_FBACK], 10);
				USART_puts(V.str);
				USART_putsr(", ");
				utoa(V.str, V.adc_data[ADC_AUX], 10);
				USART_puts(V.str);
				USART_putsr(", ");
				utoa(V.str, V.adc_data[ADC_CW], 10);
				USART_puts(V.str);
				USART_putsr(", ");
				utoa(V.str, V.adc_data[ADC_CCW], 10);
				USART_puts(V.str);
				USART_putsr(", ");
				utoa(V.str, V.adc_data[ADC_ZERO], 10);
				USART_puts(V.str);
				USART_putsr("\r\n");
				V.sequence = 0;
				V.motor_state = APP_STATE_COMMAND;
				break;
			default:
				break;
			}
		case APP_STATE_COMMAND:
			break;
		case APP_STATE_EXECUTE:
			utoa(V.str, V.adc_data[ADC_FBACK], 10);
			if (V.opto1 || V.opto2)
				USART_putsr(" AT LIMIT SWITCH: ");
			USART_putsr("Pot: ");
			USART_puts(V.str);
			if (motordata[0].pot.cal_high && motordata[0].pot.cal_low) {
				ADC_read();
				USART_putsr(" Position: ");
				itoa(V.str, motordata[0].pot.scaled_actual, 10);
				USART_puts(V.str);
			} else {
				USART_putsr(" Position: uncalibrated");
			}
			USART_putsr("\r\n");
			V.motor_state = APP_STATE_COMMAND;
			break;
		case APP_STATE_TEST:
			V.motor_state = APP_STATE_COMMAND;
			init_motor();
			run_cal();
			break;
		default:
			break;
		}
	}
}
