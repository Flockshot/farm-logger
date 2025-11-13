/*
 * UserNode.c
 *
 * Created: 26-May-23 20:12:03
 *  Author: msoma
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <util/delay.h>


#define F_CPU 8000000

#define LCD_2_LINE_MODE 0x38
#define DISPLAY_ON 0x08
#define CLEAR_DISPLAY 0x01
#define SHIFT_AFTER_DISPLAY 0x06
#define DISPLAY_BLINK 0x0C
#define DISPLAY_ON_BLINK 0x0F





void PORTS_INIT();
void USART_INIT();
void LCD_INIT();
void sendCommand(unsigned char command);
void sendData(unsigned char data);
void sendLCD(unsigned char isCommand, unsigned input);

void CHECK_INPUT();
void TRANSMIT(unsigned char message);


void SLEEP();
void CLEAR_VARS();




unsigned char totalChars = 0;
unsigned char toClear = 0;
unsigned char lastChar = 'a';


unsigned char keypad[4][3] = { {'1', '2', '3'},
							   {'4', '5', '6'},
							   {'7', '8', '9'},
							   {'.', '0', '.'}};

unsigned char interuptCount = 0;
unsigned char isStarted = 0;
unsigned char dataSent = 0x00;

int main(void)
{
	
	PORTS_INIT();
	LCD_INIT();
	USART_INIT();
	
	//Enable INT1.
	//GICR = 0x80;
	//Enable ISC11 and ISC10. Rising edge on INT 1.
	EICRA = 0x0C;
	EIMSK = 0x02;
	
	//sendData('R');
	//sendData('E');
	//sendData('S');
	//lastChar = '\0';
	//sendData('\0');
	

	isStarted = 1;
	//sei();
	while (1)
	{
		SLEEP();
		//CHECK_INPUT();
	}
}



void PORTS_INIT()
{
	/*
	DDRA.0 = RS
	DDRA.1 = RW
	DDRA.2 = E
	*/
	DDRA = 0xFF;
	
	//DATA output.
	DDRB = 0xFF;
	
	//Keyboard
	DDRC = 0xF0;
	PORTC = 0x07; // ground all rows CHIGH
}

void USART_INIT()
{
	UCSR0B = (1<<TXEN0 | 1<<TXCIE0 | 1<<RXEN0 | 1<<RXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UBRR0H = 0x00;
	UBRR0L = 0x33;	
}
void LCD_INIT()
{
	CLEAR_VARS();
	//2 Line display:
	//https://morf.lv/simple-library-for-driving-20x4-lcd-with-4bits	
	sendCommand(LCD_2_LINE_MODE);
	sendCommand(DISPLAY_ON_BLINK);
	sendCommand(CLEAR_DISPLAY);
	sendCommand(SHIFT_AFTER_DISPLAY);
}





void CHECK_INPUT()
{
	unsigned char col = -1;
	unsigned char row = -1;
	unsigned char i = 0;

	for(i=0; i<10; i++) 
	{
		_delay_ms(20); // for debounce
		col = (PINC & 0x07); // read columns
		
		if(col != 0x07)
			break;
	} 
	if(col == 0x07)
		return;

	for(i=0; i<10; i++)
	//while(1)
	{
		PORTC = 0xEF; // ground row 0
		col = (PINC & 0x07); // read columns
		if (col != 0x07) // column detected
		{
			row = 0; // save row location
			break; // exit while loop
		}
		
		PORTC = 0xDF; // ground row 1
		col = (PINC & 0x07); // read columns
		if (col != 0x07) // column detected
		{
			row = 1; // save row location
			break; // exit while loop
		}
		PORTC = 0xBF; // ground row 2
		col = (PINC & 0x07); // read columns
		if (col != 0x07) // column detected
		{
			row = 2; // save row location
			break; // exit while loop
		}
		PORTC = 0x7F; // ground row 3
		col = (PINC & 0x07); // read columns
		if (col != 0x07) // column detected
		{
			row = 3; // save row location
			break; // exit while loop
		}
	}		unsigned char toOut;		if (col == 0x03)
		toOut = (keypad[row][0]);
	else if (col == 0x5)
		toOut = (keypad[row][1]);
	else //(col == 0x06)
		toOut = (keypad[row][2]);
	
	//interuptCount=0;
	PORTC = 0x07;
	//interuptCount=1;
	sendData(toOut);
	TRANSMIT(toOut);
}






void TRANSMIT(unsigned char message)
{
	dataSent = message;
	UDR0 = message;
}



void sendCommand(unsigned char command) {
	sendLCD(0, command);
}

void sendData(unsigned char data)
{
	if((totalChars%20) == 0 && data == ' ')
		return;

	sendLCD(1, data);	
	totalChars++;
	
	unsigned char lineNum = totalChars/20;
	unsigned char currentChars = totalChars%20;
	unsigned char remainingChars = 20-currentChars;
	
	if(data == '\r' && currentChars != 0)
	{
		totalChars += remainingChars;
		lineNum++;
	}
		
	if(remainingChars==20 || data == '\r')
	{
		if(lineNum == 1)
			sendCommand(0xC0);
		else if(lineNum == 2)
			sendCommand(0x94);
		else if(lineNum == 3)
			sendCommand(0xD4);
		else if(lineNum == 4)
			LCD_INIT();			
	}
	
	//lastChar = data;
}

void sendLCD(unsigned char isData, unsigned input)
{
	PORTA = 0x00+isData;
	PORTB = input;
		
	PORTA = 0x04+isData;
	_delay_ms(1);
	PORTA = 0x00+isData;
	_delay_ms(1);
}


//Interupt for input
ISR(INT1_vect)
{
	sleep_disable();			
	CHECK_INPUT();
}


ISR(USART0_TX_vect)
{
	sleep_disable();
	
	if(dataSent == '.')
		LCD_INIT();
}

// Sample Interrupt handler for USART0 (User side) when an RX is done
ISR(USART0_RX_vect)
{
	sleep_disable();
	
	while (! (UCSR0A & (1<<RXC0))){}; // Double checking	flag
		
	unsigned char dataToSend = UDR0;
	
	if(lastChar==';')
		LCD_INIT();

	lastChar = dataToSend;
	sendData(dataToSend);
}


void SLEEP()
{
	sleep_enable(); // arm sleep mode
	sei(); // global interrupt enable
	sleep_cpu(); // put CPU to sleep
}

void CLEAR_VARS()
{
	totalChars = 0;
	toClear = 0;
	lastChar = 'a';
}