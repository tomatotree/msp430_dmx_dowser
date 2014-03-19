//
// 1 Channel DMX Servo Controller
//
// Author: Nathan Strickland
//
//
//
//
//
//
//
//
//
//
//
// MSP430G2xx3IN20 Connection Diagram:
//             __ __
//       VCC -|  U  |- GND
// TX/RX SEL -|     |- NC
//    DMX TX -|     |- NC
//    DMX RX -|     |- SBW/TEST
//     DIP 1 -|     |- SBW/RST
//     DIP 2 -|     |- NC
//     DIP 4 -|     |- SERVO PWM
//     DIP 8 -|     |- DIP 256
//    DIP 16 -|     |- DIP 128
//    DIP 32 -|_____|- DIP 64
//






// ******** Configuration Options ********

// Minimum and Maximum servo pulse width:
//     This controls the maximum servo travel in either direction. Units are .0005ms, or 2000 per 1ms.
//     Most servos are centered at 1.5ms (3000). The maximum and minimum values vary from servo to servo,
//     with ranges of 0.5-2.5ms or 1-2ms being typical. Check your servo's data sheet, or experiment by
//     trial and error to find the proper values for your servo. If you don't need the full range of motion,
//     these values can be further reduced.
#define SERVO_MIN 2000 // 2000=1ms is a reasonable default
#define SERVO_MAX 4000 // 4000=2ms

// Servo Test Mode:  **NOT IMPLEMENTED**
//     Ignores all DMX input, and instead switches between maximum and minimum servo values every 2s.
//     Useful for testing servo range of motion.
#define SERVO_TEST_MODE 0 // 1 is on, 0 is off (default)

// DIP Switch DMX Address:
//     Read DMX address from 9 dip switches. Address is one plus value of any enabled switch.
//     Refer to connection diagram above. If disabled, use constant address set below.
#define USE_DIP_SWITCHES 1 // 1 is enabled (default), 0 is off
#define STATIC_DMX_ADDRESS 512 // range 1-512; ignored if USE_DIP is enable above




// ******** End Configuration ********


#include "msp430g2203.h"

typedef unsigned char u_char;
typedef unsigned int u_int;

#define DMX_IN_PIN BIT1
#define TXRX_SEL_PIN BIT0
#define PWM_PIN BIT6

void configMSP();
void setPWM(u_char position);
void changeChannelFromDipSw();

u_char data = 0;

u_char ucaStatus = 0;
u_char rxData = 0;

#define DMX_IDLE 0
#define DMX_BREAK 1
#define DMX_START 2
#define DMX_READY 3
#define DMX_RECEIVE 4

u_char dmxStatus = 0;
u_char dmxDataReady = 0;
u_int dmxChannel = STATIC_DMX_ADDRESS;
u_int dmxCounter = 0;

u_char dmxAddressChanged = 0;

u_int pwmValue;
u_int opRange = SERVO_MAX - SERVO_MIN;
u_char pwmOn = 0;

void main(void) {

    WDTCTL = WDTPW + WDTHOLD;
    configMSP();

    while (1) {
    	if (dmxAddressChanged) {
    		dmxAddressChanged = 0;
    		changeChannelFromDipSw();
    	}
    	if (dmxDataReady) {
    		dmxDataReady = 0;
    		setPWM(data);
    		if (!pwmOn) {
    			TACTL |= MC_1; // turn on TA0 in up mode
    			pwmOn = 1;
    		}
    	}
    }
}

void changeChannelFromDipSw() {
	u_int temp = 0;
	temp |= (P2IN & (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5)); // read bits 3-8 from P2
	temp = temp << 3; //shift those bits into the right place
	temp |= ((P1IN & (BIT3|BIT4|BIT5))>>3); // add in bits 0-2 from P1, shifting them into the right place
	temp += 1; // add 1, making total range 1-512
	dmxChannel = temp;
}

void configMSP() {

    // DCO
    BCSCTL1 = CALBC1_16MHZ;
    DCOCTL = CALDCO_16MHZ;

    // Enable pull-downs on P3
    P3REN = 0xFF;

    // TA0 PWM Setup
    P1DIR |= PWM_PIN;
    P1SEL |= PWM_PIN;
    TACCTL1 = OUTMOD_7; // set on CCR0, reset on CCR1
    TACTL = TASSEL_2 + ID_3; // SMCLK/8 = 2mHz
    TACCR0 = 40000; // 2mHz/40000 = 20ms period

    // RS485 DMX Receiver Setup
    P1DIR |= TXRX_SEL_PIN;
    P1SEL |= DMX_IN_PIN;
    P1SEL2 |= DMX_IN_PIN;
    P1OUT &= ~TXRX_SEL_PIN; // Low for RX

    // DMX UART Setup
    UCA0CTL0 |= UCSPB; // 2 stop bits
    UCA0CTL1 |= UCSSEL_2 + UCRXEIE + UCBRKIE;
    UCA0BR0 = 0x40; // 16MHz/64 = 250k
    UCA0BR1 = 0;
    UCA0MCTL = 0;
    UCA0CTL1 &= ~UCSWRST;
    IE2 |= UCA0RXIE;

    if (USE_DIP_SWITCHES) {
    	// DMX Address dip switch setup
    	P1SEL &= ~(BIT3|BIT4|BIT5); // set P1.3-P1.5 as inputs
    	P1OUT |= (BIT3|BIT4|BIT5); // set resistor mode to pull-up
    	P1REN |= (BIT3|BIT4|BIT5); // enable pull-up resistors
    	P2SEL &= ~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5); // same thing for P2.0-P2.6
    	P2OUT |= (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);
    	P2REN |= (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);

    	// DMX Address dip switch interrupt setup
    	P1IES = P1IN; // read current state; set high pins to interrupt on hi->low transitions and low pins to interrupt on low->hi
    	P2IES = P2IN; // this needs to be toggled every interrupt
    	P1IFG = 0; // clear any interrupts
    	P2IFG = 0;
    	P1IE = (BIT3|BIT4|BIT5); // enable interrupts
    	P2IE = (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);

    	// Read from dip switches on first run through main
    	dmxAddressChanged = 1;
    }

    _bis_SR_register(GIE);
}

void setPWM(u_char position) {
	pwmValue = SERVO_MIN + (((position*(opRange/10))/255)*10);
	TACCR1 = pwmValue;
}

#pragma vector=PORT1_VECTOR
__interrupt void P1_ISR(void) {
	P1IES = P1IN; // set interrupt edge such that a interrupt will trigger on a change from the current state
	P1IFG = 0; // clear interrupt flag
	dmxAddressChanged = 1;
}

#pragma vector=PORT2_VECTOR
__interrupt void P2_ISR(void) {
	P2IES = P2IN; // set interrupt edge such that a interrupt will trigger on a change from the current state
	P2IFG = 0; // clear interrupt flag
	dmxAddressChanged = 1;
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

    ucaStatus = UCA0STAT;
    rxData = UCA0RXBUF;

    if (ucaStatus & UCBRK) {
        dmxStatus = DMX_BREAK;
        ucaStatus = 0;
        dmxCounter = 0;
    } else {
        switch (dmxStatus) {
        case DMX_IDLE:
            break;
        case DMX_BREAK:
            if (rxData == 0) {
                dmxStatus = DMX_START;
            } else {
                dmxStatus = DMX_IDLE;
            }
            dmxCounter++;
            break;
        case DMX_START:
            if (dmxCounter == dmxChannel) {
                dmxStatus = DMX_RECEIVE;
            } else {
                dmxCounter++;
                break;
            }
        case DMX_RECEIVE:
            data = rxData;
            dmxStatus = DMX_IDLE;
            dmxDataReady = 1;
            break;
        }
    }
}
