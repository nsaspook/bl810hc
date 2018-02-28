
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
#include "bl810hc.h"
#include "bl810hc_build.h"

volatile uint8_t sequence = 0;
struct V_data V;
volatile struct motortype motordata[1], *motor_ptr;

const uint16_t TIMEROFFSET = 40000, TIMERDEF = 15000; // flash timer 26474
const uint8_t BDELAY = 24, RUNCOUNT = 100;

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
		sequence++;
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
		PIR1bits.TMR1IF = 0;
		WRITETIMER1(TIMERDEF);
		ADCON0bits.GO = 1; // and begin A/D conv, will set adc int flag when done.
		LATDbits.LATD1 = 1;
	}

	if (PIR1bits.ADIF) { // ADC conversion complete flag
		PIR1bits.ADIF = 0;
		V.adc_data[V.adc_i] = ADRES;
		switch (V.adc_i) {
		case ADC_CW:
			if (ADRES < 8000) {
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
			}
			break;
		case ADC_CCW:
			if (ADRES < 8000) {
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

			}
			break;
		default:
			break;
		}

		if (V.adc_i++ >= MAX_ADC_CHAN) {
			V.adc_i = 0;
			V.adc_flag = true;
			V.sequence++;
		}
		LATDbits.LATD1 = 0;
	}

	if (PIR3bits.TMR4IF) { // Timer4 int handler for input debounce
		PIR3bits.TMR4IF = 0;
		PR4 = 0xff;
		if (!BUTTON1 && !V.db1--) {
			V.button1 = true;
			BLED1 = 1;
			V.blink = 0;
			V.motor_state = APP_STATE_EXECUTE;
			V.cmd_state = CMD_IDLE;
			V.bdelay = BDELAY;
			V.odelay = BDELAY;
		} else {
			if (BUTTON1)
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
			if (BUTTON2)
				V.db2 = 8;
		}

		// MCLV controller three button logic
		if (V.cmd_state != CMD_IDLE) {
			LATDbits.LATD0 = (uint8_t)!LATDbits.LATD0;
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
				break;
			case CMD_OFF:
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
			if (V.run) {
				V.run = false;
				V.cmd_state = CMD_OFF;
			}
		}
	}
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

void main(void)
{
	uint8_t z, tester[] = " 810HC Brushless motor tester ";
	V.adc_i = 0;
	V.motor_state = APP_STATE_INIT;
	V.adc_state = ADC_FBACK;
	V.cmd_state = CMD_IDLE;
	V.sequence = 0;
	V.sequence_save = 0;

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
	ADCON2bits.ACQT = 0b110;
	ADCON2bits.ADCS = 0b110;

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

	T4CON = 0b01111111;
	PR4 = 0xff;
	PIE3bits.TMR4IE = 1;

	/* Display a prompt to the USART */
	USART_putsr(build_version);

	//	while (DataRdy1USART()) { // dump rx data
	//		z = Read1USART();
	//	};
	//	while (DataRdy2USART()) { // dump rx data
	//		z = Read2USART();
	//	};

	PIR1bits.RC1IF = 0;
	PIR3bits.RC2IF = 0;
	PIR1bits.TX1IF = 0;
	PIR3bits.TX2IF = 0;
	INTCONbits.PEIE = 1;
	INTCONbits.GIEH = 1; // enable high ints

	USART_puts(tester);

	/* Loop forever */
	while (true) {
		switch (V.motor_state) {
		case APP_STATE_INIT:
			V.adc_i = 0;
			V.adc_state = ADC_FBACK;
			V.sequence = 0;
			V.sequence_save = 0;
			V.motor_state = APP_STATE_WAIT_INPUT;
			V.cmd_state = CMD_IDLE;
			BLED1 = false;
			BLED2 = false;
			ELED1 = false;
			ELED2 = false;
			break;
		case APP_STATE_WAIT_INPUT:
			switch (sequence) {
			case 1:
				S1 = 0;
				break;
			case 2:
				S1 = 1;
				break;
			case 3:
				S3 = 0;
				break;
			case 4:
				S3 = 1;
				break;
			case 5:
				S2 = 0;
				break;
			case 6:
				S2 = 1;
				break;
			case 7:
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
				break;
			case 8:
				S3 = 0;
				break;
			case 9:
				S3 = 1;
				sequence = 0;
				V.motor_state = APP_STATE_COMMAND;
				break;
			default:
				break;
			}
		case APP_STATE_COMMAND:
			break;
		case APP_STATE_EXECUTE:
			V.motor_state = APP_STATE_COMMAND;
			break;
		case APP_STATE_TEST:
			V.motor_state = APP_STATE_COMMAND;
			break;
		default:
			break;
		}
	}
}
