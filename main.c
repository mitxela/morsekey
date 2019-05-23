/* Morse Code USB Keyboard
 * based on EasyLogger from Objective Development
 * mitxela.com/projects/morse_code_usb_keyboard_mk_ii
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv.h"

/*
Pin assignment:
PB1 = waveform output to piezo
PB4 = key input (active low with pull-up)

PB0, PB2 = USB data lines
*/

#define BIT_PIEZO 1
#define BIT_KEY 4

#define UTIL_BIN4(x)        (uchar)((0##x & 01000)/64 + (0##x & 0100)/16 + (0##x & 010)/4 + (0##x & 1))
#define UTIL_BIN8(hi, lo)   (uchar)(UTIL_BIN4(hi) * 16 + UTIL_BIN4(lo))

#ifndef NULL
#define NULL    ((void *)0)
#endif

/* ------------------------------------------------------------------------- */

const uchar     dashlength  = 9;
const uchar     spacelength = 45;   // longer for farnsworth pause

static uchar    reportBuffer[2];    // buffer for HID reports
static uchar    idleRate;           // in 4 ms units

static uchar    modifier;

static uchar    valueBuffer[16];
static uchar    *nextDigit;

static uchar    symbolBuffer[16];
static uchar    *symbol;


/* ------------------------------------------------------------------------- */

const PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                 // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                 // USAGE (Keyboard)
    0xa1, 0x01,                 // COLLECTION (Application)
    0x05, 0x07,                 // USAGE_PAGE (Keyboard)
    0x19, 0xe0,                 // USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                 // USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                 // LOGICAL_MINIMUM (0)
    0x25, 0x01,                 // LOGICAL_MAXIMUM (1)
    0x75, 0x01,                 // REPORT_SIZE (1)
    0x95, 0x08,                 // REPORT_COUNT (8)
    0x81, 0x02,                 // INPUT (Data,Var,Abs)
    0x95, 0x01,                 // REPORT_COUNT (1)
    0x75, 0x08,                 // REPORT_SIZE (8)
    0x25, 0x65,                 // LOGICAL_MAXIMUM (101)
    0x19, 0x00,                 // USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                 // USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                 // INPUT (Data,Ary,Abs)
    0xc0                        // END_COLLECTION
};
/* We use a simplified keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* Keyboard usage values, see usb.org's HID-usage-tables document, chapter
 * 10 Keyboard/Keypad Page for more codes.
 */
#define MOD_CONTROL_LEFT    (1<<0)
#define MOD_SHIFT_LEFT      (1<<1)
#define MOD_ALT_LEFT        (1<<2)
#define MOD_GUI_LEFT        (1<<3)
#define MOD_CONTROL_RIGHT   (1<<4)
#define MOD_SHIFT_RIGHT     (1<<5)
#define MOD_ALT_RIGHT       (1<<6)
#define MOD_GUI_RIGHT       (1<<7)

/* ------------------------------------------------------------------------- */

static void buildReport(void) {
    uchar   key = 0;
    uchar   mod = 0;

    if(nextDigit != NULL){
        key = *nextDigit;
        if (key) mod = modifier;
    }
    reportBuffer[0] = mod;
    reportBuffer[1] = key;
}

static void typechar(char key) {
    if (nextDigit==&valueBuffer[sizeof(valueBuffer)]) {*--nextDigit = 0xff;}/* terminate with 0xff */
    *--nextDigit = 0;
    *--nextDigit = key;
}

static inline void soundOn(){
  TCCR0A = 0b00010010;
}
static inline void soundOff(){
  TCCR0A = 0;
}

static void timerPoll(void) {
    static uchar up = 255;
    static uchar down;

    if(TIFR & (1 << TOV1)){
        TIFR = (1 << TOV1);  // clear overflow

        if(!(PINB & (1 << BIT_KEY))){ //key held
            soundOn();
            if (down++ ==spacelength+dashlength) {
                down=spacelength;
                typechar(0x2A);
                up=255;
            }
            if (down<spacelength) up=0;
        } else if (down){
            soundOff();
            if (symbol!=&symbolBuffer[sizeof(symbolBuffer)]){
                if (down>=spacelength) {
                    *symbol=0;
                    while (symbol!=&symbolBuffer[0]) {*--symbol=0;}
                } else if (down<dashlength) {
                    *symbol++ = '.';
                } else {
                    *symbol++ = '-';
                }
            }
            down =0;
        } else {
            soundOff();
            if (up++ ==255) up=255;
            if (up==1+dashlength) {
                modifier=0;
                //work out key
                if (symbolBuffer[0]=='.') {
                    if (symbolBuffer[1]=='.'){
                        if (symbolBuffer[2]=='.'){
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]=='.'){
                                    if (symbolBuffer[5]==0){
                                        typechar(0x22);//5
                                    }
                                } else if (symbolBuffer[4]=='-') {
                                    if (symbolBuffer[5]==0){
                                        typechar(0x21);//4
                                    }
                                } else {
                                    typechar(0x0B);//h
                                }
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]=='.'){
                                    //if (symbolBuffer[5]==0){
                                        //typechar();//
                                    //}
                                } else if (symbolBuffer[4]=='-') {
                                    if (symbolBuffer[5]==0){
                                        typechar(0x20);//3
                                    }
                                } else {
                                    typechar(0x19);//V
                                }
                            } else {
                                typechar(0x16);//s
                            }
                        } else if (symbolBuffer[2]=='-') {
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]==0) {
                                    typechar(0x09);//F
                                }
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]=='-' && symbolBuffer[5]==0) {
                                    typechar(0x1F);//2
                                } else if (symbolBuffer[4]=='.' && symbolBuffer[5]=='.' && symbolBuffer[6]==0) {
                                    modifier = MOD_SHIFT_RIGHT;
                                    typechar(0x38);// /
                                }
                            } else {
                                typechar(0x18);//U
                            }
                        } else {
                            typechar(0x0C);//i
                        }
                    } else if (symbolBuffer[1]=='-') {
                        
                        if (symbolBuffer[2]=='.'){
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]==0) typechar(0x0F);//L
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]=='.' && symbolBuffer[5]=='-' && symbolBuffer[6]==0) typechar(0x37);//.
                                else if (symbolBuffer[4]=='.' && symbolBuffer[6]==0) {modifier=MOD_SHIFT_RIGHT;typechar(0x2E);}//+
                            } else {
                                typechar(0x15);//R
                            }
                        } else if (symbolBuffer[2]=='-') {
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]==0) {
                                    typechar(0x13);//P
                                }
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]==0) {
                                    typechar(0x0D);//J
                                } else if (symbolBuffer[4]=='-' && symbolBuffer[5]==0){
                                    typechar(0x1E);//1
                                }
                            } else {
                                typechar(0x1A);//W
                            }
                        } else {
                            typechar(0x04);//A
                        }
                    } else {
                        typechar(0x08);//e
                    }
                } else if (symbolBuffer[0]=='-') {
                    if (symbolBuffer[1]=='.'){
                        if (symbolBuffer[2]=='.'){
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]=='.'){
                                    if (symbolBuffer[5]==0){
                                        typechar(0x23);//6
                                    }
                                } else if (symbolBuffer[4]=='-') {
                                    if (symbolBuffer[5]==0){
                                        typechar(0x2E);// =
                                    }
                                } else {
                                    typechar(0x05);//B
                                }
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]=='.'){
                                    if (symbolBuffer[5]==0){
                                        typechar(0x38);// /
                                    }
                                } else if (symbolBuffer[4]=='-') {
                                    //if (symbolBuffer[5]==0){
                                        //typechar();//
                                    //}
                                } else {
                                    typechar(0x1B);//X
                                }
                            } else {
                                typechar(0x07);//D
                            }
                        } else if (symbolBuffer[2]=='-') {
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]==0) {
                                    typechar(0x06);//C
                                }
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]==0) {
                                    typechar(0x1C);//Y
                                }
                            } else {
                                typechar(0x0E);//K
                            }
                        } else {
                            typechar(0x11);//N
                        }
                    } else if (symbolBuffer[1]=='-') {
                        if (symbolBuffer[2]=='.'){
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]==0) typechar(0x1D);//Z
                                else if (symbolBuffer[4]=='.' && symbolBuffer[5]==0) typechar(0x24);//7
                                else if (symbolBuffer[4]=='-' && symbolBuffer[5]=='-' && symbolBuffer[6]==0) typechar(0x36);//,
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]==0) typechar(0x14);//Q
                            } else {
                                typechar(0x0A);//G
                            }
                        } else if (symbolBuffer[2]=='-') {
                            if (symbolBuffer[3]=='.'){
                                if (symbolBuffer[4]=='.' && symbolBuffer[5]==0) {
                                    typechar(0x25);//8
                                }
                            } else if (symbolBuffer[3]=='-') {
                                if (symbolBuffer[4]=='-' && symbolBuffer[5]==0) {
                                    typechar(0x27);// 0
                                } else if (symbolBuffer[4]=='.' && symbolBuffer[5]==0) {
                                    typechar(0x26);// 9
                                }
                            } else {
                                typechar(0x12);//O
                            }
                        } else {
                            typechar(0x10);//M
                        }
                    } else {
                        typechar(0x17);//T
                    }
                }
                
                //empty stack
                *symbol=0;
                while (symbol!=&symbolBuffer[0]) {*--symbol=0;}
            }
            if (up==spacelength && down<spacelength) {
                //type space
                typechar(0x2C);
            }
        }

    }
}


/* ------------------------------------------------------------------------- */

static void timerInit(void) {
    TCCR0B = 3;
    OCR0A = 64;
    TCCR1 = 0x0b;           /* select clock: 16.5M/1k -> overflow rate = 16.5M/256k = 62.94 Hz */
    modifier=0;
    symbol=&symbolBuffer[sizeof(symbolBuffer)];
    while (symbol!=&symbolBuffer[0]) {*--symbol=0;}
}


/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar   usbFunctionSetup(uchar data[8]) {
    usbRequest_t    *rq = (void *)data;

    usbMsgPtr = reportBuffer;
    if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS){  /* class request type */
        if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
            /* we only have one report type, so don't look at wValue */
            buildReport();
            return sizeof(reportBuffer);
        }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
            usbMsgPtr = &idleRate;
            return 1;
        }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
            idleRate = rq->wValue.bytes[1];
        }
    }else{
        /* no vendor specific requests implemented */
    }
    return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void) {
    uchar   step = 128;
    uchar   trialValue = 0, optimumValue;
    int     x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

    /* do a binary search: */
    do{
        OSCCAL = trialValue + step;
        x = usbMeasureFrameLength();    /* proportional to current real frequency */
        if(x < targetValue)             /* frequency still too low */
            trialValue += step;
        step >>= 1;
    }while(step > 0);
    /* We have a precision of +/- 1 for optimum OSCCAL here */
    /* now do a neighborhood search for optimum value */
    optimumValue = trialValue;
    optimumDev = x; /* this is certainly far away from optimum */
    for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++){
        x = usbMeasureFrameLength() - targetValue;
        if(x < 0)
            x = -x;
        if(x < optimumDev){
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
    /* Disable interrupts during oscillator calibration since
     * usbMeasureFrameLength() counts CPU cycles.
     */
    cli();
    calibrateOscillator();
    sei();
    eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */

int main(void) {
    uchar   i;
    uchar   calibrationValue;

    calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
    if(calibrationValue != 0xff){
        OSCCAL = calibrationValue;
    }
    usbDeviceDisconnect();
    for(i=0;i<20;i++){  /* 300 ms disconnect */
        _delay_ms(15);
    }
    usbDeviceConnect();
    DDRB |= 1 << BIT_PIEZO;   /* output for buzzer */
    PORTB |= 1 << BIT_KEY;  /* pull-up on key input */
    wdt_enable(WDTO_1S);
    timerInit();
    usbInit();
    sei();
    for(;;){    /* main event loop */
        wdt_reset();
        usbPoll();
        if(usbInterruptIsReady() && nextDigit != NULL){ /* we can send another key */
            buildReport();
            usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
            if(*++nextDigit == 0xff)    /* this was terminator character */
                nextDigit = NULL;
        }
        timerPoll();
    }
    return 0;
}