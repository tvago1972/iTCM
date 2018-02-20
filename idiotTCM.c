/*

	Idiot Transmission Control Module (iTCM)
	Version: 0.11	2015-11-09
	Author:	Thomas A. Vago

	The purpose of this program is to integrate a 6-speed manual transmission into the vehicle electronic network of a 2005 Dodge
Magnum SXT that was originally equipped with a 42RLE 4-speed electronically controlled automatic transmission.

	The iTCM shall initially pass gearshift position codes to the vehicle's existing TCM (which is integrated into the vehicle's
PCM) by simulating the switch output of a 42RLE Transmission Range Sensor (TRS). The iTCM shall also pass a generated input speed
sensor signal and output speed sensor signal to the vehicle's existing TCM.

	After this is accomplished, the iTCM program shall be modified to incorporate CAN-C transceiver functionality, so as to
directly communicate over the vehicle's network bus. Initially, this will allow the program to listen to the network so as to
capture TCM network packets that correspond to gearshift signals and transmission-specific speed signals.

	At some later date, the iTCM program shall be further modified to send gearshift position and transmission speed network
packets, and the vehicle's existing TCM shall be then disabled.

	History:
	0.1	2015-11-07	Initial capability. Program written to simulate TRS. Infinite loop that simulates
				P-R-N-OD-3-L-3-OD-N-R-P TRS states, with 1 second between gears, and 0.1 seconds between
				valid "in-between-gear" TRS states. Debug output to SCI serial interface.
	0.11	2015-11-09	Modified PORTD outputs to have system-off state TRS signal have code for Park. Added PORTC
				switch state inputs, and switch debouncing code. Removed demonstration shifter infinite loop.
				Added logic to generate valid TRS state, depending on input switch states.
	0.12	2017-12-13	cleaned up code to make it easier to understand shifter switch states. Reconfigured SCI support
				to be dependent on #ifdef
	0.13	2017-12-17	moved pin assignments for switches to deconflict with SDA/SCL. Re-wrote switch logic to use table

inputs:

AVR	Arduino
pin	pin	description
---	---	-----------
C3	26/A3	clutch safety interlock switch (normally open, grounded)
C2	25/A2	clutch upstop switch (normally closed, grounded)
C1	24/A1	reverse gear select switch (normally open, grounded)
C0	23/A0	parking brake switch (normally open, grounded)

transceiver pins:

AVR	Arduino
pin	pin	description
---	---	-----------
C4/SDA	27/A4	TWI data
C5/SCL	28/A5	TWI clock
D0/RXD	2/0	SCI receive
D1/TXD	3/1	SCI send
B1	15/9	CANBUS shield chip select
B3/MOSI	17/11	CANBUS shield MOSI SCI data out
B4/MISO	18/12	CANBUS shield MISO SCI data in
B5/SCK	19/13	CANBUS shield SCK SCI clock
D2/INT0	4/2	CANBUS shield interrupt pin

outputs:

AVR	Arduino
pin	pin	description
---	---	-----------
D7	13/7	TRS T41 switch sense (P/N switch)
D6	12/6	TRS T42 switch sense
D5	11/5	TRS T3 switch sense
D4	6/4	TRS T1 switch sense
?	?	turbine speed sensor signal
?	?	output speed sensor signal

*/

//#define debugOutput true
//#define hasParkingBrake true
//#define usesRelayShield true

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

void read_modify_write(uint8_t * addr, uint8_t and_mask, uint8_t or_mask);
int main(void);

const uint8_t processorSpeed = 16; // processor speed in MHz
#ifdef debugOutput
const unsigned int myubbr = (unsigned int)(processorSpeed * 625ul / 96ul - 1);
#endif // debugOutput
const unsigned long t2CyclesPerSecond = (unsigned long)(processorSpeed * 15625ul); // (processorSpeed * 1000000 / (timer 0 prescaler))
const unsigned int switchDelay = (unsigned int)(5ul * t2CyclesPerSecond / 25600ul);

const uint8_t sensorT41 =		(1 << PORTD7);
const uint8_t sensorT42 =		(1 << PORTD6);
const uint8_t sensorT3 =		(1 << PORTD5);
const uint8_t sensorT1 =		(1 << PORTD4);

const uint8_t switchNeutralPin =	(1 << PINC3);
const uint8_t switchUpstopPin =		(1 << PINC2);
const uint8_t switchReversePin =	(1 << PINC1);
#ifdef hasParkingBrake
const uint8_t switchParkBrakePin =	(1 << PINC0);
#endif // hasParkingBrake

#ifdef hasParkingBrake
const uint8_t switchDDRbitMask =	((1 << DDC3) | (1 << DDC2) | (1 << DDC1) | (1 << DDC0));
const uint8_t switchPortBitMask =	((1 << PORTC3) | (1 << PORTC2) | (1 << PORTC1) | (1 << PORTC0));
const uint8_t switchISRbitMask =	((1 << PCINT11) | (1 << PCINT10) | (1 << PCINT9) | (1 << PCINT8));
const uint8_t switchPinBitMask =	(switchNeutralPin | switchUpstopPin | switchReversePin | switchParkBrakePin);
#else // hasParkingBrake
const uint8_t switchDDRbitMask =	((1 << DDC3) | (1 << DDC2) | (1 << DDC1));
const uint8_t switchPortBitMask =	((1 << PORTC3) | (1 << PORTC2) | (1 << PORTC1));
const uint8_t switchISRbitMask =	((1 << PCINT11) | (1 << PCINT10) | (1 << PCINT9));
const uint8_t switchPinBitMask =	(switchNeutralPin | switchUpstopPin | switchReversePin);
#endif // hasParkingBrake

const uint8_t shifterPositionCount = 11;

const uint8_t shifterPatternPark =	(sensorT41 | sensorT42 | sensorT3);
const uint8_t shifterPatternT1 =	(sensorT42 | sensorT3);
const uint8_t shifterPatternReverse =	(sensorT42);
const uint8_t shifterPatternT2 =	(sensorT42 | sensorT1);
const uint8_t shifterPatternNeutral =	(sensorT41 | sensorT42 | sensorT1);
const uint8_t shifterPatternOverdrive =	(sensorT1);
const uint8_t shifterPatternT3 =	(sensorT3 | sensorT1);
const uint8_t shifterPatternGear3 =	(sensorT3);
const uint8_t shifterPatternGearL =	(sensorT42 | sensorT3 | sensorT1);

const uint8_t shifterDDRbitMask =	((1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4));
const uint8_t shifterPortBitMask =	(sensorT41 | sensorT42 | sensorT3 | sensorT1);
#ifdef usesRelayShield
const uint8_t shifterPortXORmask =	0;
#endif // usesRelayShield

const uint8_t shifterIndexPark =	0;
const uint8_t shifterIndexT1 =		1;
const uint8_t shifterIndexReverse =	2;
const uint8_t shifterIndexT2 =		3;
const uint8_t shifterIndexNeutral =	4;
const uint8_t shifterIndexDrive =	6;
const uint8_t shifterIndexLow =		10;

volatile uint8_t shifterPositionIndex;

volatile uint8_t shifterPositionSetting[(unsigned int)(shifterPositionCount)] =
{ // contains valid shifter codes for a Chrysler 42RLE transmission
	shifterPatternPark,		// Park
	shifterPatternT1,		// T1
	shifterPatternReverse,		// Reverse
	shifterPatternT2,		// T2
	shifterPatternNeutral,		// Neutral
	shifterPatternT2,		// T2
	shifterPatternOverdrive,	// Overdrive
	shifterPatternT3,		// T3
	shifterPatternGear3,		// 3d gear
	shifterPatternT3,		// T3
	shifterPatternGearL		// Low gear
};

#ifdef hasParkingBrake
volatile uint8_t switchPositionSetting[16] =
{ // contains switch states
	shifterIndexPark		// parking brake   set, reverse switch   set, clutch upstop   set, neutral safety   set
	,shifterIndexPark		// parking brake clear, reverse switch   set, clutch upstop   set, neutral safety   set
	,shifterIndexNeutral		// parking brake   set, reverse switch clear, clutch upstop   set, neutral safety   set
	,shifterIndexNeutral		// parking brake clear, reverse switch clear, clutch upstop   set, neutral safety   set
	,shifterIndexPark		// parking brake   set, reverse switch   set, clutch upstop clear, neutral safety   set
	,shifterIndexPark		// parking brake clear, reverse switch   set, clutch upstop clear, neutral safety   set
	,shifterIndexNeutral		// parking brake   set, reverse switch clear, clutch upstop clear, neutral safety   set
	,shifterIndexNeutral		// parking brake clear, reverse switch clear, clutch upstop clear, neutral safety   set
	,shifterIndexT1			// parking brake   set, reverse switch   set, clutch upstop   set, neutral safety clear
	,shifterIndexReverse		// parking brake clear, reverse switch   set, clutch upstop   set, neutral safety clear
	,shifterIndexT2			// parking brake   set, reverse switch clear, clutch upstop   set, neutral safety clear
	,shifterIndexLow		// parking brake clear, reverse switch clear, clutch upstop   set, neutral safety clear
	,shifterIndexT1			// parking brake   set, reverse switch   set, clutch upstop clear, neutral safety clear
	,shifterIndexReverse		// parking brake clear, reverse switch   set, clutch upstop clear, neutral safety clear
	,shifterIndexPark		// parking brake   set, reverse switch clear, clutch upstop clear, neutral safety clear
	,shifterIndexDrive		// parking brake clear, reverse switch clear, clutch upstop clear, neutral safety clear
};
#else // hasParkingBrake
volatile uint8_t switchPositionSetting[16] =
{ // contains switch states
	shifterIndexPark		// parking brake   set, reverse switch   set, clutch upstop   set, neutral safety   set
	,shifterIndexPark		// parking brake clear, reverse switch   set, clutch upstop   set, neutral safety   set
	,shifterIndexNeutral		// parking brake   set, reverse switch clear, clutch upstop   set, neutral safety   set
	,shifterIndexNeutral		// parking brake clear, reverse switch clear, clutch upstop   set, neutral safety   set
	,shifterIndexPark		// parking brake   set, reverse switch   set, clutch upstop clear, neutral safety   set
	,shifterIndexPark		// parking brake clear, reverse switch   set, clutch upstop clear, neutral safety   set
	,shifterIndexNeutral		// parking brake   set, reverse switch clear, clutch upstop clear, neutral safety   set
	,shifterIndexNeutral		// parking brake clear, reverse switch clear, clutch upstop clear, neutral safety   set
	,shifterIndexReverse		// parking brake   set, reverse switch   set, clutch upstop   set, neutral safety clear
	,shifterIndexReverse		// parking brake clear, reverse switch   set, clutch upstop   set, neutral safety clear
	,shifterIndexLow		// parking brake   set, reverse switch clear, clutch upstop   set, neutral safety clear
	,shifterIndexLow		// parking brake clear, reverse switch clear, clutch upstop   set, neutral safety clear
	,shifterIndexReverse		// parking brake   set, reverse switch   set, clutch upstop clear, neutral safety clear
	,shifterIndexReverse		// parking brake clear, reverse switch   set, clutch upstop clear, neutral safety clear
	,shifterIndexDrive		// parking brake   set, reverse switch clear, clutch upstop clear, neutral safety clear
	,shifterIndexDrive		// parking brake clear, reverse switch clear, clutch upstop clear, neutral safety clear
};
#endif // hasParkingBrake

const uint8_t sShifterSet =		0b10000000;
const uint8_t sShiftInProgress =	0b01000000;
const uint8_t sReadSwitchState =	0b00100000;

volatile uint8_t shifterStatus;
volatile uint8_t shifterTargetPosition;
volatile uint8_t shifterPosition;
volatile uint8_t lastPINCstate;
volatile uint8_t switchState;

volatile unsigned int switchCount;

typedef union
{

	unsigned int ui;
	uint8_t u8[2];

} union16;

#ifdef debugOutput
namespace usart
{

	const uint8_t outputBufferLength = 32;
	const uint8_t inputBufferLength = 16;

	volatile uint8_t bufferStatus;

	const uint8_t outputBufferFull =	0b10000000;
	const uint8_t outputBufferEmpty =	0b01000000;
	const uint8_t inputBufferFull =		0b00100000;
	const uint8_t receiveFramingError =	0b00010000;
	const uint8_t receiveDataOverrun =	0b00001000;
	const uint8_t receiveParityError =	0b00000100;
	const uint8_t inputBufferEmpty =	0b00000010;
	const uint8_t inputBufferOverrun =	0b00000001;

	volatile uint8_t outputBuffer[(unsigned int)(outputBufferLength)];
	volatile uint8_t outputBufferStart;
	volatile uint8_t outputBufferEnd;

	volatile uint8_t inputBuffer[(unsigned int)(inputBufferLength)];
	volatile uint8_t inputBufferStart;
	volatile uint8_t inputBufferEnd;

	void init(void);
	void wordOut(unsigned int wrd);
	void byteOut(uint8_t byt);
	void nybbleOut(uint8_t nyb);
	void flashOut(const char * str);
	void stringOut(char * str);
	void charOut(char chr);
	char charIn(void);

};

#endif // debugOutput
/*

Timer 0 overflow is primarily used for system time-keeping. One overflow interrupt is generated
approximately once every 1.024 milliseconds (assuming a 16 MHz clock), hereafter referred
to as a 'tick'.

*/
ISR( TIMER0_OVF_vect )
{

	static uint16_t tickCount;
	static uint8_t shifterCurrentPosition = 0;

	if (switchCount) // if there is a switch debounce countdown in progress
	{

		switchCount--; // bump down the switch debounce count by one

		if (switchCount == 0) // if switch state has been read, go pass it on to the main program
		{

			// figure out current switch state
			switchState = lastPINCstate & switchPinBitMask;
			shifterStatus |= (sReadSwitchState);

		}

	}

	if (shifterStatus & sShiftInProgress)
	{

		if (tickCount)
		{

			tickCount--;

		}
		else
		{

			tickCount = switchDelay;

			if (shifterCurrentPosition == shifterTargetPosition)
			{

				shifterStatus &= ~(sShiftInProgress);

			}
			else
			{

				if (shifterCurrentPosition > shifterTargetPosition) shifterCurrentPosition--;

				if (shifterCurrentPosition < shifterTargetPosition) shifterCurrentPosition++;

				shifterPosition = shifterPositionSetting[(unsigned int)(shifterCurrentPosition)];
#ifdef usesRelayShield
				shifterPosition ^= shifterPortXORmask;
#endif // usesRelayShield
				read_modify_write(&PORTD, shifterPortBitMask, shifterPosition);

			}

		}

	}
	else
	{

		if (shifterStatus & sShifterSet)
		{

			shifterStatus &= ~(sShifterSet);
			shifterStatus |= sShiftInProgress;
			tickCount = 0;

		}

	}

}

/*


PCINT1 pin change interrupt service routine

*/
ISR( PCINT1_vect )
{

	static uint8_t p;
	static uint8_t q;

	p = PINC; // read current pin C state
	q = p ^ lastPINCstate; // detect any changes from the last time this ISR is called

	// set switch debounce count, and let system timer handle the debouncing
	if (q & switchPinBitMask) switchCount = switchDelay;

	lastPINCstate = p; // remember the current pin C state for the next time this ISR gets called

}

#ifdef debugOutput
/*

USART serial input receive complete interrupt service routine

*/
ISR( USART_RX_vect )
{

	uint8_t i;

	i = (UCSR0A & ((1 << FE0) | (1 << DOR0) | (1 << UPE0)));
	usart::inputBuffer[(unsigned int)(usart::inputBufferStart++)] = UDR0;
	usart::inputBufferStart %= usart::inputBufferLength;

	if (usart::bufferStatus & usart::inputBufferFull)
	{

		if ((++usart::inputBufferEnd) == usart::inputBufferLength) usart::inputBufferEnd = 0;
		i |= usart::inputBufferOverrun;

	}

	if (usart::inputBufferStart == usart::inputBufferEnd) i |= usart::inputBufferFull;
	read_modify_write(&usart::bufferStatus, (usart::inputBufferEmpty | usart::receiveFramingError | usart::receiveDataOverrun | usart::receiveParityError), i);

}

/*

USART serial output data register empty interrupt service routine - handles bulk of transmitting serial characters.

*/
ISR( USART_UDRE_vect )
{

	uint8_t i;

	if (usart::bufferStatus & usart::outputBufferEmpty) read_modify_write(&UCSR0B, (1 << UDRIE0), 0); // Disable transmit buffer empty interrupt
	else
	{

		UDR0 = usart::outputBuffer[(unsigned int)(usart::outputBufferEnd++)];
		if (usart::outputBufferEnd == usart::outputBufferLength) usart::outputBufferEnd = 0;
		if (usart::outputBufferEnd == usart::outputBufferStart) i = usart::outputBufferEmpty;
		read_modify_write(&usart::bufferStatus, usart::outputBufferFull, i);

	}

}

/*

USART serial output transmission complete interrupt service routine - should only get called when all output characters have
been transmitted out.

*/
ISR( USART_TX_vect )
{

	read_modify_write(&UCSR0B, ((1 << TXCIE0) | (1 << TXEN0)), 0); // Disable transmitter and transmit complete interrupt

}

void usart::init(void)
{

	uint8_t oldSREG;

	oldSREG = SREG; // save interrupt flag status
	cli(); // disable interrupts

	bufferStatus = (outputBufferEmpty | inputBufferEmpty);
	inputBufferStart = 0;
	outputBufferStart = 0;
	inputBufferEnd = 0;
	outputBufferEnd = 0;

	SREG = oldSREG; // restore interrupt flag status

}

void usart::wordOut(unsigned int wrd)
{

	union16 * wrdPtr = (union16 *)(&wrd);

	byteOut(wrdPtr->u8[1]);
	byteOut(wrdPtr->u8[0]);

}

void usart::byteOut(uint8_t byt)
{

	nybbleOut(byt >> 4);
	nybbleOut(byt);

}

void usart::nybbleOut(uint8_t nyb)
{

	nyb &= 0x0F;
	nyb |= 0x30;
	if (nyb > 0x39) nyb += 7;

	charOut((char)(nyb));

}

void usart::flashOut(const char * str)
{

	uint8_t chr;

	while ((chr = pgm_read_byte(str++)) != 0) charOut((char)(chr));

}

void usart::stringOut(char * str)
{

	while (*str) charOut(*str++);

}

void usart::charOut(char chr)
{

	uint8_t i;
	uint8_t oldSREG;

	while (bufferStatus & outputBufferFull); // if buffer is full, then wait until buffer is no longer full

	oldSREG = SREG; // save interrupt flag status
	cli(); // disable interrupts

	outputBuffer[(unsigned int)(outputBufferStart++)] = chr;
	if (outputBufferStart == outputBufferLength) outputBufferStart = 0;
	if (outputBufferStart == outputBufferEnd) i = outputBufferFull;
	read_modify_write(&bufferStatus, outputBufferEmpty, i);
 	// Enable transmitter and interrupts
 	read_modify_write(&UCSR0B, 0, ((1 << TXCIE0) | (1 << UDRIE0) | (1 << TXEN0)));

	SREG = oldSREG; // restore interrupt flag status

}

char usart::charIn(void)
{

	uint8_t i;
	uint8_t chr;
	uint8_t oldSREG;

	while (bufferStatus & inputBufferEmpty); // if buffer is empty, then wait until buffer is no longer empty

	oldSREG = SREG; // save interrupt flag status
	cli(); // disable interrupts

	chr = inputBuffer[(unsigned int)(inputBufferEnd++)];
	if (inputBufferEnd == inputBufferLength) inputBufferEnd = 0;
	if (inputBufferEnd == inputBufferStart) i = inputBufferEmpty;
	read_modify_write(&bufferStatus, inputBufferFull, i);

	SREG = oldSREG; // restore interrupt flag status

	return chr;

}

#endif // debugOutput
void read_modify_write(volatile uint8_t * addr, uint8_t and_mask, uint8_t or_mask)
{

	uint8_t oldSREG;
	uint8_t value;

	oldSREG = SREG; // save interrupt flag status
	cli(); // disable interrupts

	value = (* addr);
	value &= ~(and_mask);
	value |= (or_mask);
	* addr = value;

	SREG = oldSREG; // restore interrupt flag status

}

int main(void)
{

	uint8_t oldSREG;
#ifdef debugOutput
	uint8_t chr;
#endif // debugOutput

	cli(); // disable interrupts

	// configure timer 0:
	// * 8-bit fast PWM mode
	// * prescaler of 64
	// * clear all interrupt flags
	// * disable channel A and B output compare interrupts
	// * enable overflow interrupt
	read_modify_write(&TCCR0A, ((1 << 3) | (1 << 2) | (1 << COM0A1) | (1 << COM0A0) | (1 << COM0B1) | (1 << COM0B0)), ((1 << WGM01) | (1 << WGM00)));
	read_modify_write(&TCCR0B, ((1 << 5) | (1 << 4) | (1 << FOC0A) | (1 << FOC0B) | (1 << WGM02) | (1 << CS02)), ((1 << CS01) | (1 << CS00)));
	read_modify_write(&TIMSK0, ((1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (1 << OCIE0B) | (1 << OCIE0A)), (1 << TOIE0));
	read_modify_write(&TIFR0, ((1 << 7) | (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3)), ((1 << OCF0B) | (1 << OCF0A) | (1 << TOV0)));

	// configure port C:
	//
	// port C pins 2, 3, 4, and 5 are used for switched ground input
	//
	read_modify_write(&DDRC, (switchDDRbitMask), 0); // configure pins C5, C4, C3, and C2 as input pins
	read_modify_write(&PORTC, (switchPortBitMask), (switchPortBitMask)); // enable pullup resistors for pins C5, C4, C3, and C2
	read_modify_write(&PCMSK1, (switchISRbitMask), (switchISRbitMask)); // select pins C5, C4, C3, and C2 for interrupts
	read_modify_write(&PCICR, (1 << PCIE1), (1 << PCIE1)); // enable selected pin C interrupts

	lastPINCstate = PINC; // initialize last PINC state value so as to not detect false switch states on start
	switchState = lastPINCstate & switchPinBitMask;

	// configure port D:
	//
	// port D pins 4, 5, 6, and 7 are used for output relay control
	//
	read_modify_write(&DDRD, (shifterDDRbitMask), (shifterDDRbitMask)); // configure pins D4, D5, D6, and D7 for output

#ifdef debugOutput
	// configure RS232 USART
	// * set asynchronous communication speed to 9600 baud
	// * set data format to 8 bits, no parity, 1 stop bit
	// * disable serial transmit pin
	// * enable serial receive pin
	// * disable double-speed asynchronous communication speed
	// * disable multi-processor mode
	// * enable receive complete interrupt
	// * disable transmit complete, transmit buffer empty interrupts
	// * clear transmit complete, receive complete, transmit buffer empty status flags
	UBRR0H = (uint8_t)(myubbr >> 8);
	UBRR0L = (uint8_t)(myubbr);
	read_modify_write(&UCSR0A, ((1 << FE0) | (1 << DOR0) | (1 << UPE0) | (1 << U2X0) | (1 << MPCM0)), (1 << TXC0));
	read_modify_write(&UCSR0B, ((1 << TXCIE0) | (1 << UDRIE0) | (1 << TXEN0) | (1 << UCSZ02) | (1 << TXB80)), ((1 << RXCIE0) | (1 << RXEN0)));
	read_modify_write(&UCSR0C, ((1 << UMSEL01) | (1 << UMSEL00) | (1 << UPM01) | (1 << UPM00) | (1 << USBS0) | (1 << UCPOL0)), ((1 << UCSZ01)| (1 << UCSZ00)));

#endif // debugOutput
	shifterStatus |= (sReadSwitchState); // signal main loop to process initial switch state

#ifdef debugOutput
	usart::init();

#endif // debugOutput
	shifterTargetPosition = 0; // set initial shift position to Park

	sei(); // enable interrupts

	while (true)
	{

		if (shifterStatus & sReadSwitchState)
		{

			oldSREG = SREG; // save interrupt flag status
			cli(); // disable interrupts

			shifterStatus &= ~(sReadSwitchState); // clear shifter switch state flag
			shifterTargetPosition = switchPositionSetting[(unsigned int)(switchState)];
			shifterStatus |= sShifterSet;

			SREG = oldSREG; // restore interrupt flag status

		}

#ifdef debugOutput
		if (!(usart::bufferStatus & usart::inputBufferEmpty)) chr = usart::charIn();

		if (chr == '\\')
		{

			usart::flashOut(PSTR("shifterStatus = "));
			usart::byteOut(shifterStatus);
			usart::charOut(',');
			usart::charOut(' ');

			usart::flashOut(PSTR("shifterTargetPosition = "));
			usart::byteOut(shifterTargetPosition);
			usart::charOut(',');
			usart::charOut(' ');

			usart::flashOut(PSTR("shifterPosition = "));
			usart::byteOut(shifterPosition);
			usart::charOut(',');
			usart::charOut(' ');

			usart::flashOut(PSTR("switchState = "));
			usart::byteOut(switchState);
			usart::charOut(0x0D);
			usart::charOut(0x0A);

			chr = 0;

		}

#endif // debugOutput
	}

	return 0;

}
