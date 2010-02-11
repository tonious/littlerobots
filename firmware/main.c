/*
 * Little Robots Firmware
 *
 * Portions of this file are from the hid-data demo by Christian Starkjohann,
 * the 1-Key Keyboard project by Flip Van Der Berg;
 *
 */

void    usbEventResetReady(void);

#include <string.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>  /* for sei() */
#include <util/delay.h>     /* for _delay_ms() */
#include <avr/eeprom.h>

#include <avr/pgmspace.h>   /* required by usbdrv.h */
#include "usbdrv.h"
#include "oddebug.h"        /* This is also an example for using debug macros */

/* ------------------------------------------------------------------------- */
/* ----------------------------- USB interface ----------------------------- */
/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[22] = {    /* USB report descriptor */
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x80,                    //   REPORT_COUNT (128)
    0x09, 0x00,                    //   USAGE (Undefined)
    0xb2, 0x02, 0x01,              //   FEATURE (Data,Var,Abs,Buf)
    0xc0                           // END_COLLECTION
};
/* Since we define only one feature report, we don't use report-IDs (which
 * would be the first byte of the report). The entire report consists of 128
 * opaque data bytes.
 */

#define	FEAT_POSITION		0x01
#define FEAT_CALIBRATE_LOW	0x02
#define FEAT_CALIBRATE_HIGH	0x03
#define FEAT_BUTTON_PRESSES	0x04


/* The following variables store the status of the current data transfer */
static uchar  bytesRemaining;
static uchar	currentCommand;

/* What should we set our servo angle to? */
static uchar	angle;

/* Have we seen any usb activity yet? */
static uchar	usb_activity;

/* Button monitoring overhead. */
static uchar	button_presses;
static uchar	button_state;
static uchar	debounce_over;
#define BUTTON_PORT	PORTB
#define BUTTON_PIN	PINB
#define BUTTON_BIT	PB3

/* We have a few variables stashed in eeprom... */
#define CALIBRATE_LOW	((uint8_t *)0x01)
#define CALIBRATE_HIGH  ((uint8_t *)0x02)
#define TIMER_CALIBRATE ((uint8_t *)0x03)

/* ------------------------------------------------------------------------- */

/* usbFunctionRead() is called when the host requests a chunk of data from
 * the device. For more information see the documentation in usbdrv/usbdrv.h.
 */
unsigned char usbFunctionRead(uchar *data, uchar len) {
	switch( currentCommand ) {
	case FEAT_POSITION:
		data[0] = angle;
		break;
	case FEAT_CALIBRATE_LOW:
		data[0] = eeprom_read_byte( CALIBRATE_LOW );
		break;
	case FEAT_CALIBRATE_HIGH:
		data[0] = eeprom_read_byte( CALIBRATE_HIGH );
		break;
	case FEAT_BUTTON_PRESSES:
		data[0] = button_presses;
		button_presses = 0;
		break;
	}

	if(len > bytesRemaining)
		len = bytesRemaining;
	bytesRemaining -= len;

	return( len );
}

/* usbFunctionWrite() is called when the host sends a chunk of data to the
 * device. For more information see the documentation in usbdrv/usbdrv.h.
 */
uchar   usbFunctionWrite(uchar *data, uchar len) {
	if(bytesRemaining == 0)
		return 1;
	if(len > bytesRemaining)
		len = bytesRemaining;

	switch( currentCommand ) {
	case FEAT_POSITION:
		angle = data[0];
		break;
	case FEAT_CALIBRATE_LOW:
		eeprom_write_byte( CALIBRATE_LOW, data[0] );
		break;
	case FEAT_CALIBRATE_HIGH:
		eeprom_write_byte( CALIBRATE_HIGH, data[0] );
		break;
	}

	bytesRemaining -= len;

	return bytesRemaining == 0; /* return 1 if this was the last chunk */
}

/* ------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
	usbRequest_t *rq = (void *)data;

	if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS) {    /* HID class request */
		if(rq->bRequest == USBRQ_HID_GET_REPORT) {
			/* wValue: ReportType (highbyte), ReportID (lowbyte) */
			/* since we have only one report type, we can ignore the report-ID */

			currentCommand = rq->wValue.bytes[0];
			bytesRemaining = 1;

			return USB_NO_MSG;  /* use usbFunctionRead() to obtain data */

		} else if(rq->bRequest == USBRQ_HID_SET_REPORT) {

			currentCommand = rq->wValue.bytes[0];
			bytesRemaining = 1;

			return USB_NO_MSG;  /* use usbFunctionWrite() to receive data from host */
		}
 	} else {
	        /* ignore vendor type requests, we don't use any */
	}
	return 0;
}

/* ------------------------------------------------------------------------- */

void pwm_set( int x ) {
	OCR0B = x;
}

void pwm_init( void ) {
	// Set non-inverting output mode on pin OC0B.
	TCCR0A = ( 1 << COM0B1 ) | ( 0 << COM0B0 );

	// Set fast pwm mode.
	TCCR0A |= ( 1 << WGM01 ) | ( 1 << WGM00 );
	TCCR0B = ( 0 << WGM02 );

	// Use /1024 Prescaler.
	// This sets a PWM frequency of ~48Hz,
	// which is just about perfect for a servo.
	TCCR0B |= ( 1 << CS02 ) | ( 0 << CS01 ) | ( 1 << CS00 );
}

/* ------------------------------------------------------------------------- */

/* This button state stuff is taken from the 1-Key-Keyboard project.
 */ 
void timerPoll( void ) {
	static unsigned int timerCnt;
	if(TIFR & (1 << TOV0)) {
		TIFR = (1 << TOV0); 
		if(++timerCnt >= 2) { // 2/50 sec delay for switch debouncing
			timerCnt = 0;
			debounce_over = 1; 
		}
	}
}

void button_update() {
        uchar tempButtonValue = bit_is_clear( BUTTON_PIN, BUTTON_BIT ); //status of switch is stored in tempButtonValue 
        if (tempButtonValue != button_state && debounce_over == 1){ //if status has changed and the debounce-delay is over
                button_state = tempButtonValue;  // change buttonState to new state
                debounce_over = 0; // debounce timer starts

		if( button_state == 0 )
			button_presses++;
        }
}

/* ------------------------------------------------------------------------- */

/* From the 1-Key-Keyboard.
 *
 * Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for * experimental purposes only!
 */
static void calibrateOscillator(void) {
	uchar step = 128;
	uchar trialValue = 0, optimumValue;
	int x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

	/* do a binary search: */
	do {
		OSCCAL = trialValue + step;
		x = usbMeasureFrameLength();    /* proportional to current real frequency */
		if(x < targetValue)             /* frequency still too low */
			trialValue += step;
		step >>= 1;
	}while( step > 0 );

	/* We have a precision of +/- 1 for optimum OSCCAL here */
	/* now do a neighborhood search for optimum value */
	optimumValue = trialValue;
	optimumDev = x; /* this is certainly far away from optimum */
	for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++) {
		x = usbMeasureFrameLength() - targetValue;
		if(x < 0)
			x = -x;
		if( x < optimumDev ) {
			optimumDev = x;
			optimumValue = OSCCAL;
		}
	}
	OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void usbEventResetReady(void) {
	calibrateOscillator();
	eeprom_write_byte( (uint8_t *)TIMER_CALIBRATE, OSCCAL );   /* store the calibrated value in EEPROM */
}


/* ------------------------------------------------------------------------- */

void startup_seek( void ) {	
	angle = eeprom_read_byte( CALIBRATE_HIGH );
	pwm_set( angle );
	_delay_ms( 400 );

	angle = eeprom_read_byte( CALIBRATE_LOW );
	pwm_set( angle );
	_delay_ms( 400 );

	angle = (eeprom_read_byte( CALIBRATE_LOW ) + eeprom_read_byte( CALIBRATE_HIGH ) ) / 2;
	pwm_set( angle );
}

int main(void)
{
	uchar   i;

	// If we have a stored timer calibration, let's use it.
	uchar calibrationValue = eeprom_read_byte( (uint8_t *)TIMER_CALIBRATE );
	if(calibrationValue != 0xff){
		OSCCAL = calibrationValue;
	}

	// Init watchdog and USB.
	wdt_enable(WDTO_1S);
	usbInit();

	// Zero out some global variables.
	button_presses = 0;
	usb_activity = 0;

	// Init, and seek the servo a bit, to show we've booted.
	DDRB |= _BV(PB4) | _BV( PB1 ) | _BV( BUTTON_PIN ); // Inputs.
	BUTTON_PORT |= _BV(BUTTON_BIT); // Pull up for switch.

	pwm_init();
	startup_seek();

	// Force re-enumeration, do this while interrupts are disabled! 
	usbDeviceDisconnect();  
	i = 0;
	while(--i) { /* fake USB disconnect for > 250 ms */
		wdt_reset();
		_delay_ms(1);
	}
	usbDeviceConnect();
	sei();

	// Main loop.
	for(;;) { 
		wdt_reset();
		usbPoll();

		pwm_set( angle );
		button_update();
		timerPoll();
	}
	return 0;
}

