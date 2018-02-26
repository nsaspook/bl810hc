
// PIC18F8723 Configuration Bit Settings

#include <p18f8723.h>

// CONFIG1H
#pragma config OSC = HSPLL      // Oscillator Selection bits (HS oscillator, PLL enabled (Clock Frequency = 4 x FOSC1))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Two-Speed Start-up disabled)

// CONFIG2L
#pragma config PWRT = ON       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown-out Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = ON         // Watchdog Timer (WDT enabled)
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
#pragma config XINST = ON   // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

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




#include <usart.h>
#include <delays.h>
#include <timers.h>
#include <stdlib.h>
#include <EEP.h>
#include <GenericTypeDefs.h>
#include "vtouch.h"
#include "vtouch_build.h"

void rxtx_handler(void);

volatile uint16_t timer0_off = TIMEROFFSET;
volatile uint8_t sequence = 0;

#pragma code touch_int = 0x8

void touch_int(void)
{
	_asm goto rxtx_handler _endasm
}
#pragma code

#pragma interrupt rxtx_handler

void rxtx_handler(void) // all timer & serial data transform functions are handled here
{
	static uint8_t junk = 0, c = 0, *data_ptr,
		i = 0, data_pos, data_len;

	if (INTCONbits.RBIF) {
		junk = PORTB;
		PORTD = junk;
		INTCONbits.RBIF = 0;
	}

	/* start with data_ptr pointed to address of data, data_len to length of data in bytes, data_pos to 0 to start at the beginning of data block */
	/* then enable the interrupt and wait for the interrupt enable flag to clear
	/* send buffer and count xmit data bytes for host link */
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
		WriteTimer0(timer0_off);
		LATHbits.LATH0 = !LATHbits.LATH0; // flash onboard led
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
}

void wdtdelay(uint32_t delay)
{
	uint32_t dcount;
	for (dcount = 0; dcount <= delay; dcount++) { // delay a bit
		ClrWdt(); // reset the WDT timer
	};
}

void Cylon_Eye(uint8_t invert)
{
	static uint8_t cylon = 0xfe, LED_UP = TRUE;
	static int32_t alive_led = 0xfe;

	if (invert) { // screen status feedback
		LATD = ~cylon; // roll LEDs cylon style
	} else {
		LATD = cylon; // roll leds cylon style (inverted)
	}

	if (LED_UP && (alive_led != 0)) {
		alive_led = alive_led * 2;
		cylon = cylon << 1;
	} else {
		if (alive_led != 0) alive_led = alive_led / 2;
		cylon = cylon >> 1;
	}
	if (alive_led < 2) {
		alive_led = 2;
		LED_UP = TRUE;
	} else {
		if (alive_led > 128) {
			alive_led = 128;
			LED_UP = FALSE;
		}
	}
}

void main(void)
{
	uint8_t z;
	INTCON = 0;
	INTCON3bits.INT1IE = 0;
	INTCON3bits.INT2IE = 0;
	INTCON3bits.INT3IE = 0;
	// default interface
	/* Configure  PORT pins for output */
	TRISA = 0;
	LATA = 0;
	TRISG = 0;
	LATG = 0;
	LATGbits.LATG3 = 1;
	LATGbits.LATG4 = 1;
	/* check for touchscreen configuration data and setup switch on port B */

	INTCON2bits.RBPU = 1;
	LATB = 0xff;
	TRISB = 0b00110000; // read LEDs and set switches
	INTCONbits.RBIE = 1;

	TRISC = 0;
	LATC = 0;
	TRISD = 0;
	TRISE = 0;
	LATE = 0xFF;
	TRISF = 0;
	LATF = 0xFF;
	TRISH = 0;
	LATH = 0;
	TRISJ = 0;

	/*
	 * Open the USART configured as
	 * 8N1, 9600 baud, in /transmit/receive INT mode
	 */
	/* Host */
	Open1USART(USART_TX_INT_ON &
		USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

	/* TouchScreen */
	Open2USART(USART_TX_INT_OFF &
		USART_RX_INT_ON &
		USART_ASYNCH_MODE &
		USART_EIGHT_BIT &
		USART_CONT_RX &
		USART_BRGH_LOW, 64); // 40mhz osc HS		9600 baud

	OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_256);
	WriteTimer0(timer0_off); //	start timer0 at 1 second ticks

	/* Display a prompt to the USART */
	putrs1USART(build_version);

	while (DataRdy1USART()) { // dump rx data
		z = Read1USART();
	};
	while (DataRdy2USART()) { // dump rx data
		z = Read2USART();
	};

	/* Enable interrupt priority */
	RCONbits.IPEN = 1;
	PIR1bits.RCIF = 0;
	PIR3bits.RC2IF = 0;
	PIR1bits.TX1IF = 0;
	PIE1bits.TX1IE = 0;
	PIR3bits.TX2IF = 0;
	INTCONbits.GIEL = 0; // disable low ints
	INTCONbits.GIEH = 1; // enable high ints

	/* Loop forever */
	while (TRUE) {
		ClrWdt(); // reset the WDT timer
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
		case 9:
			S2 = 0;
			break;
		case 10:
			S2 = 1;
			break;
		case 20:
			sequence=0;
			break;
		default:
			break;
		}
	}
}
