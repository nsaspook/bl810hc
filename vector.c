#include "vector.h"

static const uint16_t ADC_TRIGGER = 500;
static const uint8_t BDELAY = 4, RUNCOUNT = 20; // button low time , motor run time for knob click
const uint16_t TIMEROFFSET = 40000, TIMERDEF = 61000, TIMER3REG = 15600; // timer3 value for 10ms clock
const uint8_t TIMER4DEF = 0xD0;

void interrupt high_priority tm_handler(void) // all timer & serial data transform functions are handled here
{
	static uint8_t c = 0, *data_ptr,
		i = 0, data_pos, data_len;

	if (INTCONbits.RBIF) {
		V.b_data = PORTB;
		INTCONbits.RBIF = 0;
	}

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	 send buffer and count xmit data bytes for host link */
	if (PIE1bits.TX1IE && PIR1bits.TX1IF) { // send data to host USART
		if (data_pos >= data_len) { // buffer has been sent
			if (TXSTA1bits.TRMT) { // last bit has been shifted out
				PIE1bits.TX1IE = 0; // stop data xmit
			}
		} else {
			TXREG1 = *data_ptr; // send data and clear PIR1bits.TX1IF
			data_pos++; // move the data pointer
			data_ptr++; // move the buffer pointer position
		}
	}

	if (PIR1bits.RCIF) {
		if (RCSTA1bits.OERR) {
			RCSTA1bits.CREN = 0; //	clear overrun
			RCSTA1bits.CREN = 1; // re-enable
		}
		c = RCREG1; // read from host
	}

	if (INTCONbits.TMR0IF) { // check timer0 irq 1 second timer
		//check for TMR0 overflow
		INTCONbits.TMR0IF = 0; //clear interrupt flag
		WRITETIMER0(TIMEROFFSET);
		LATHbits.LATH0 = (uint8_t)!LATHbits.LATH0; // flash onboard led
		// check the LED blink flags
		if (V.blink & 0b00000001) BLED1 = (uint8_t)!BLED1;
		if (V.blink & 0b00000010) BLED2 = (uint8_t)!BLED2;
		if (V.blink & 0b00000100) ELED1 = (uint8_t)!ELED1;
		if (V.blink & 0b00001000) ELED2 = (uint8_t)!ELED2;
		V.sequence++;
	}

	if (PIR3bits.RC2IF) {
		if (RCSTA2bits.OERR) {
			RCSTA2bits.CREN = 0; //	clear overrun
			RCSTA2bits.CREN = 1; // re-enable
		}
		/* Get the character received from the USART */
		c = RCREG2;
	}

	if (PIR1bits.TMR1IF) { // Timer1 int handler
		ADCON0bits.CHS = V.adc_i;
		ADCON0bits.GO = 1; // and begin A/D conv, will set adc int flag when done.
		PIR1bits.TMR1IF = 0;
		WRITETIMER1(TIMERDEF);
		LATDbits.LATD2 = (uint8_t)!LATDbits.LATD2;

		if (V.buzzertime == 0u) {
			if (V.buzzer_on)
				ALARMO = !ALARMO;
		} else {
			ALARMO = !ALARMO;
			V.buzzertime--;
		}

	}

	if (PIR1bits.ADIF) { // ADC conversion complete flag
		PIR1bits.ADIF = 0;
		V.adc_data[V.adc_i] = ADRES;
		switch (V.adc_i) {
		case ADC_CW:
			if (ADRES < ADC_TRIGGER) {
				if (motordata[0].pot.cw) {
					if (!V.cw) // only trigger once per switching sequence
						motordata[0].pot.cw--; // trigger on second adc read
				} else {
					motordata[0].pot.cw = 1;
					V.run = true;
					V.runcount = RUNCOUNT;
					V.cw = true;
					V.ccw = false;
					V.blink = 0b00000010;
					BLED1 = false;
					BLED2 = true;
					V.motor_state = APP_STATE_EXECUTE;
					if (V.cmd_state == CMD_IDLE)
						V.cmd_state = CMD_CW;
					V.bdelay = BDELAY;
					V.odelay = BDELAY;
					LATDbits.LATD1 = 1;
					if (!motordata[0].pot.cal_high && !motordata[0].pot.cal_low)
						V.buzzertime = 20;
				}
			} else {
				motordata[0].pot.cw = 1;
				V.cw = false;
			}
			break;
		case ADC_CCW:
			if (ADRES < ADC_TRIGGER) {
				if (motordata[0].pot.ccw) {
					if (!V.ccw)
						motordata[0].pot.ccw--;
				} else {
					motordata[0].pot.ccw = 1;
					V.run = true;
					V.runcount = RUNCOUNT;
					V.cw = false;
					V.ccw = true;
					V.blink = 0b00000001;
					BLED1 = true;
					BLED2 = false;
					V.motor_state = APP_STATE_EXECUTE;
					if (V.cmd_state == CMD_IDLE)
						V.cmd_state = CMD_CCW;
					V.bdelay = BDELAY;
					V.odelay = BDELAY;
					LATDbits.LATD1 = 1;
					if (!motordata[0].pot.cal_high && !motordata[0].pot.cal_low)
						V.buzzertime = 20;
				}
			} else {
				motordata[0].pot.ccw = 1;
				V.ccw = false;
			}
			break;
		default:
			break;
		}

		if (V.adc_i++ >= MAX_ADC_CHAN) { // the last ADC AN4 is connected to signal ground
			V.adc_i = 0;
			V.adc_flag = true;
			V.motor_current_tmp += V.adc_data[ADC_MOTOR];
			if (V.m_avg++ >=64) {
				V.m_avg = 0;
				V.motor = (uint16_t) (V.motor_current_tmp >> 6);
				V.motor_current_tmp = 0;
			}
			V.sequence++;
		}
	}

	if (PIR2bits.TMR3IF) { //      Timer3 int handler
		PIR2bits.TMR3IF = 0; // clear int flag
		WRITETIMER3(TIMER3REG);
		LATDbits.LATD3 = (uint8_t)!LATDbits.LATD3;
		LATDbits.LATD0 = 0;
		V.clock10++;

		if (V.clock10_set) {
			if (V.clock10_count) {
				V.clock10_count--;
			} else {
				V.clock10_set = false;
			}
		}

		if (V.clock10_setD) {
			if (V.clock10_countD) {
				V.clock10_countD--;
			} else {
				V.clock10_setD = false;
			}
		}


		// constrain set limits
		if (motordata[0].pot.pos_set < 0)
			motordata[0].pot.pos_set = 0;
		if (motordata[0].pot.pos_set > ACTUAL)
			motordata[0].pot.pos_set = ACTUAL;
		if (motordata[0].pot.scaled_set < 0)
			motordata[0].pot.scaled_set = 0;
		if (motordata[0].pot.scaled_set > SCALED)
			motordata[0].pot.scaled_set = SCALED;

		LATDbits.LATD6 = OPTO1;
		LATDbits.LATD7 = OPTO2;
	}

	if (PIR3bits.TMR4IF) { // Timer4 int handler for input debounce
		PIR3bits.TMR4IF = 0;
		PR4 = TIMER4DEF;

		/* push buttons */
		if (!BUTTON1 && !V.db1--) { // one trigger per low state
			V.button1 = true;
			BLED1 = 1;
			V.blink = 0;
			V.motor_state = APP_STATE_TEST;
			V.cmd_state = CMD_IDLE;
			V.bdelay = BDELAY;
			V.odelay = BDELAY;
		} else {
			if (BUTTON1) // only reset trigger when signal goes high again
				V.db1 = 8;
		}
		if (!BUTTON2 && !V.db2--) {
			V.button2 = true;
			BLED2 = 1;
			V.blink = 0;
			V.motor_state = APP_STATE_EXECUTE;
			if (V.cmd_state == CMD_IDLE)
				V.cmd_state = CMD_OFF;
			V.bdelay = BDELAY;
			V.odelay = BDELAY;
		} else {
			if (BUTTON2) // keep button pressed flag(s) until another process clears
				V.db2 = 8;
		}

		/* limit flags */
		if (OPTO1 && !V.db3--) { // opto sensor LOW when false, HIGH when blocked by flag
			V.opto1 = true; // set limit trigger flag, OPTO1 is the true limit state
			BLED1 = 1;
			V.blink = 0;
			if (D1 && V.cw) {
				V.motor_state = APP_STATE_EXECUTE;
				if (V.cmd_state == CMD_IDLE)
					V.cmd_state = CMD_OFF;
			}
			V.bdelay = BDELAY;
			V.odelay = BDELAY;
		} else {
			if (!OPTO1) { // only reset trigger when signal goes low again
				V.opto1 = false; // clear flag away from limits
				V.db3 = 4;
			}
		}
		if (OPTO2 && !V.db4--) {
			V.opto2 = true;
			BLED2 = 1;
			V.blink = 0;
			if (D1 && V.ccw) {
				V.motor_state = APP_STATE_EXECUTE;
				if (V.cmd_state == CMD_IDLE)
					V.cmd_state = CMD_OFF;
			}
			V.bdelay = BDELAY;
			V.odelay = BDELAY;
		} else {
			if (!OPTO2) {
				V.opto2 = false; // clear flag away from limits
				V.db4 = 4;
			}
		}

		// MCLV controller three button logic state machine
		if (V.cmd_state != CMD_IDLE) {
			LATDbits.LATD1 = 0;

			switch (V.cmd_state) {
			case CMD_CW:
				if (!D2) { // need to switch to cw mode
					if (V.bdelay--) {
						S2 = 0;
					} else {
						S2 = 1;
						V.cmd_state = CMD_IDLE;
					}
				} else {
					V.cmd_state = CMD_ON;
				}
				LATDbits.LATD0 = 1;
				break;
			case CMD_CCW:
				if (D2) { // need to switch to ccw mode
					if (V.bdelay--) {
						S2 = 0;
					} else {
						S2 = 1;
						V.cmd_state = CMD_IDLE;
					}
				} else {
					V.bdelay = BDELAY;
					V.odelay = BDELAY;
					V.cmd_state = CMD_ON;
				}
				LATDbits.LATD0 = 1;
				break;
			case CMD_OFF:
				//				LATDbits.LATD0 = (uint8_t)!LATDbits.LATD0;
				if (!V.odelay--) { // delay before press
					V.odelay = 0;
					if (D1) { // need to switch to OFF mode
						if (V.bdelay--) {
							S3 = 0;
						} else {
							S3 = 1;
							V.cmd_state = CMD_IDLE;
						}
					} else {
						V.cmd_state = CMD_IDLE;
					}
				}
				LATDbits.LATD0 = 1;
				break;
			case CMD_ON:
				if (!V.odelay--) { // delay before press
					V.odelay = 0;
					if (!D1) { // need to switch to ON mode
						if (V.bdelay--) {
							S3 = 0;
						} else {
							S3 = 1;
							V.cmd_state = CMD_IDLE;
						}
					} else {
						V.cmd_state = CMD_IDLE;
					}
				}
				LATDbits.LATD0 = 1;
				break;
			default:
				V.cmd_state = CMD_IDLE;
				V.odelay = BDELAY;
				V.bdelay = BDELAY;
				break;
			}
		} else {
			S1 = 1;
			S2 = 1;
			S3 = 1;
			V.odelay = BDELAY;
			V.bdelay = BDELAY;
		}
		if (V.runcount && V.run) {
			V.runcount--;
		} else {
			if (V.run && !V.testing) {
				V.run = false;
				V.cmd_state = CMD_OFF;
			}
		}
	}
}

uint32_t clock10(void)
{
	uint32_t ret;

	//	INTCONbits.GIEH = 0;
	ret = V.clock10;
	//	INTCONbits.GIEH = 1;
	return ret;
}

void run_ccw(void)
{
	V.run = true;
	V.testing = true;
	V.runcount = RUNCOUNT;
	V.cw = false;
	V.ccw = true;
	V.blink = 0b00000001;
	BLED1 = true;
	BLED2 = false;
	V.motor_state = APP_STATE_EXECUTE;
	if (V.cmd_state == CMD_IDLE)
		V.cmd_state = CMD_CCW;
	V.bdelay = BDELAY;
	V.odelay = BDELAY;
}

void run_cw(void)
{
	V.run = true;
	V.testing = true;
	V.runcount = RUNCOUNT;
	V.cw = true;
	V.ccw = false;
	V.blink = 0b00000010;
	V.opto1 = false;
	V.opto2 = false;
	BLED1 = false;
	BLED2 = true;
	V.motor_state = APP_STATE_EXECUTE;
	if (V.cmd_state == CMD_IDLE)
		V.cmd_state = CMD_CW;
	V.bdelay = BDELAY;
	V.odelay = BDELAY;
}

void run_stop(void)
{
	V.testing = false;
	V.blink = 0;
	V.motor_state = APP_STATE_EXECUTE;
	if (V.cmd_state == CMD_IDLE)
		V.cmd_state = CMD_OFF;
	V.bdelay = BDELAY;
	V.odelay = BDELAY;
}

bool is_cw(void)
{
	return D2;
}

bool is_run(void)
{
	return D1;
}