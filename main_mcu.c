/*
 * main.c
 *
 * Created: 21-May-23 18:49:07
 * Author : msoma
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

#define MST_WD_MENU "Enter MS WD Choice:\r1-30ms\r2-250ms\r3-500ms \0"
#define SLV_WD_MENU "Enter SL WD Choice:\r1-500ms\r2-1000ms\r3-2000ms \0"

#define SENSOR_RST_INF_MSG "\rSensor reset ;"

#define USER_MENU "Enter choice:\r1-Mem Dump \r2-Last Entry \r3-Restart \0"

#define USER 0
#define SENSOR 1

#define CRC_POLY 0xD4
#define ONES 0xFF
#define ZEROS 0x00
#define MEM_START 0x500


// Maximum buffer sizes for transmission:
#define USER_TR_BUFFER_SIZE 128
#define SENSOR_TR_BUFFER_SIZE 5
#define STACK_SIZE 100


void MEMORY_INIT();
void BUFFER_INIT();
void USER_BUFFER_INIT();
void SENSOR_BUFFER_INIT();
//Buffer *CREATE_BUFFER(unsigned char size);
void USART_INIT(unsigned char port);
void CONFIG_MASTER_WD();
void ENABLE_MASTER_WD(uint16_t timer);
void CONFIG_SLAVE_WD();
void SLAVE_WD_ENABLE(uint16_t timer);
void SENSOR_INIT();
void APPLY_CRC3(unsigned char input);
unsigned char CRC3(unsigned char input);
unsigned char CRC11(unsigned char input, unsigned char tos);

void HANDLE_MENU_INPUT();
void DUMP_MEMORY();
void LAST_ENTRY();
unsigned char *CONVERT_TO_ASCII(unsigned char toConvert, unsigned char *ascii);

void HANDLE_SENSOR_INPUT(unsigned char input);
void LOG();
unsigned char IS_LOG(unsigned char packet);
unsigned char IS_REPEAT(unsigned char packet);
unsigned char IS_ACKNOWLEDGE(unsigned char packet);
void REPEAT_REQUEST();
unsigned char CRC11_CHECK(unsigned char input, unsigned char tos);
unsigned char CRC3_CHECK(unsigned char input);
unsigned char IS_COMMAND(unsigned char packet);
unsigned char IS_DATA(unsigned char packet);



void TRANSMIT(unsigned char buffer);
void MESSAGE_USER(const unsigned char *message);
void MESSAGE_SENSOR(const unsigned char *message);
unsigned char TAKE_USER_INPUT();
void ADD_TO_BUFFER(unsigned char buffer, const unsigned char *toAdd);
void SLEEP();

void PUSH(unsigned char val);
unsigned char POP();
unsigned char TOS();



unsigned char userBuffer[USER_TR_BUFFER_SIZE] = "";
unsigned char userBufferSize = 0; // keeps track of character index in buffer
unsigned char userBufferIndex = 0;
unsigned char userBufferCurrent = ' '; // reception one character at a time
	
unsigned char sensorBuffer[SENSOR_TR_BUFFER_SIZE] = "";
unsigned char sensorBufferSize = 0; // keeps track of character index in buffer
unsigned char sensorBufferIndex = 0;
unsigned char sensorBufferCurrent = ' '; // reception one character at a time

// Data memory pointers
unsigned char *x; // points to the next data memory entry
unsigned char *z; // used in memory dumps


unsigned char *STACK;

unsigned char isStarted = 0;
unsigned char current = 'n';
//Stack* myStack = createStack();

unsigned char crc3Var = 'x';


int main(void)
{  
	while (1)
	{
	  
	   /*
	   eeprom_write_byte(0x0000, 0xFF);
	   eeprom_write_byte(0x0001, 0xFF);
	   eeprom_write_byte(0x0002, 0xFF);
	   eeprom_write_byte(0x0003, 0xFF);
	   eeprom_write_byte(0x0004, 0xFF);
	   eeprom_write_byte(0x0005, 0xFF);
	   */
		//unsigned int *x = 0x0500;
		MCUCR = 0x80;
		//tester = eeprom_read_word((const uint16_t *) 0x0000);

		sleep_enable(); // arm sleep mode
		sei(); // global interrupt enable
		
		MEMORY_INIT();

		BUFFER_INIT();
		
		USART_INIT(USER);
		USART_INIT(SENSOR);
		
		CONFIG_MASTER_WD();
		CONFIG_SLAVE_WD();
		
		SENSOR_INIT();
		
		MESSAGE_USER(USER_MENU);
		isStarted = 1;
		
		while (isStarted)
		{
			SLEEP();
		}
	}    
}

void MEMORY_INIT()
{
	x = (unsigned char *) MEM_START; // points to the next data memory entry
	z = (unsigned char *) MEM_START; // used in memory dumps
	STACK = (unsigned char *) (0x1100 - STACK_SIZE);
}



void BUFFER_INIT()
{
	USER_BUFFER_INIT();
	SENSOR_BUFFER_INIT();
}

void USER_BUFFER_INIT()
{
	userBuffer[USER_TR_BUFFER_SIZE] = "";
	strcpy(userBuffer, "");
	userBufferSize = 0; // keeps track of character index in buffer
	userBufferIndex = 0;
	userBufferCurrent = ' '; // reception one character at a time
}
void SENSOR_BUFFER_INIT()
{
	sensorBuffer[SENSOR_TR_BUFFER_SIZE] = "";
	strcpy(sensorBuffer, "");
	sensorBufferSize = 0; // keeps track of character index in buffer
	sensorBufferIndex = 0;
	sensorBufferCurrent = ' '; // reception one character at a time
}

void USART_INIT(unsigned char port)
{
	if(port == 1)
	{
		//USART1
		//DDRD = 1<<3;
		//PORTD = 0<<2;
		
		UCSR1B = (1<<TXEN1| 1<<TXCIE1| 1<<RXEN1| 1<<RXCIE1);
		UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
		UBRR1H = 0x00;
		UBRR1L = 0x33;
	}
	else if(port == 0)
	{
		//USART0
		//DDRE = 1<<3;
		//PORTE.2 = 0b0;
		
		//Turn on Recieve, Transmission and their interupts
		UCSR0B = (1<<TXEN0 | 1<<TXCIE0 | 1<<RXEN0 | 1<<RXCIE0);
		UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
		UBRR0H = 0x00;
		UBRR0L = 0x33;
	}
}



void CONFIG_MASTER_WD()
{
	uint16_t timer = eeprom_read_word(0x0000);
	
	
	
	if(timer == 0xFFFF)
	{
		MESSAGE_USER(MST_WD_MENU);
		unsigned char userInput = ' ';
		
		while(!(userInput == '1' || userInput == '2' || userInput == '3'))
		userInput = TAKE_USER_INPUT();
		
		if(userInput == '1')
		timer = 0x001E;
		else if(userInput == '2')
		timer = 0x00FA;
		else if(userInput == '3')
		timer = 0x01F4;
		
		//unsigned char byte_2 = timer;
		//unsigned char byte_1 = (timer>>8);
		//eeprom_write_byte(0x0000, byte_1);
		//eeprom_write_byte(0x0001, byte_2); 
		eeprom_write_word(0x0000, timer);
		eeprom_write_byte(0x0010, 0xFF);
		
	}
	//ENABLE_MASTER_WD(timer);
}
void ENABLE_MASTER_WD(uint16_t timer)
{
	if(timer == 0x001E)
	wdt_enable(WDTO_30MS);
	else if(timer == 0x00FA)
	wdt_enable(WDTO_250MS);
	else if(timer == 0x01F4)
	wdt_enable(WDTO_500MS);
}

void CONFIG_SLAVE_WD()
{
	uint16_t timer = eeprom_read_word(0x0002);
	
	if(timer == 0xFFFF)
	{
		MESSAGE_USER(SLV_WD_MENU);
		unsigned char userInput = ' ';
		
		while(!(userInput == '1' || userInput == '2' || userInput == '3'))
		userInput = TAKE_USER_INPUT();
		
		if(userInput == '1')
		timer = 0x01F4;
		else if(userInput == '2')
		timer = 0x03E8;
		else if(userInput == '3')
		timer = 0x07D0;
		
		//unsigned char byte_2 = timer;
		//unsigned char byte_1 = (timer>>8);
		//eeprom_write_byte(0x0002, byte_1);
		//eeprom_write_byte(0x0003, byte_2);
		eeprom_write_word(0x0003, timer);
		eeprom_write_byte(0x0010, 0xFF);
		
	}
	//SLAVE_WD_ENABLE(timer);
}
void SLAVE_WD_ENABLE(uint16_t timer)
{
	//500 or 1000 or 2000
	//counter value
	TCNT1 = 0;
	TCCR1A = 0x00;
	TCCR1B = 0b00001100; // CTC with 256 prescalling
	
	// ms/us = *1000;
	OCR1A = (timer*1000/32);
	
	TIMSK = (1<<OCIE1A);
}

void SENSOR_INIT()
{
	APPLY_CRC3(0x00);
	PUSH(crc3Var);
	unsigned char pop = POP();
	 _delay_ms(11);
	MESSAGE_SENSOR(&pop);
	//MESSAGE_USER(SENSOR_RST_INF_MSG);
}

void APPLY_CRC3(unsigned char input)
{
	crc3Var = CRC3(input);
	input &= 0b11100000;
	crc3Var |= input;
	
	//return packet;
}

unsigned char CRC3(unsigned char input)
{
	unsigned char crc = input;
	int counter;
	for(counter = 0; counter<3; counter++)
	{
		if(crc >= 0x80) //most significant bit is 1.
		crc = crc ^ CRC_POLY;
		
		crc = (crc<<1);
	}
	crc = crc>>3;

	return crc;
}

//unsigned char current_crc = -1;

unsigned char CRC11(unsigned char input, unsigned char tos)
{
	unsigned char crc = tos;
	unsigned char packet = input;
	//current_crc = crc;
	int counter;
	for(counter = 0; counter<11; counter++)
	{
		if(crc >= 0x80)
		crc = crc ^ CRC_POLY;
		
		crc = (crc<<1);
		unsigned char temp = (0b10000000 & packet);
		packet = packet<<1;
		crc = crc | (temp>>7); 
	}
	//TODO MAKE CHANGE 
	crc = crc>>3;
	//current_crc = crc;
	return crc;
}

void HANDLE_MENU_INPUT()
{
	unsigned char input = userBuffer[userBufferSize-2];
	
	USER_BUFFER_INIT();
	
	if(input == '1')
		DUMP_MEMORY();
	else if(input == '2')
		LAST_ENTRY();
	else if(input == '3')
		isStarted = 0;
	
	USER_BUFFER_INIT();
}

//ADD CHECK FOR STACK SKIP IT.
void DUMP_MEMORY()
{
	z = MEM_START;
	while(z < x)
	{
		unsigned char toPrint = *z;
		unsigned char inASCII[2] = "";
		CONVERT_TO_ASCII(toPrint, inASCII);
		MESSAGE_USER(inASCII);
		z++;
	}
}
void LAST_ENTRY() {
	unsigned char toPrint = *(x-1);
	unsigned char inASCII[2] = "";
	CONVERT_TO_ASCII(toPrint, inASCII);
	MESSAGE_USER(inASCII);
	//MESSAGE_USER((x-1));
}

unsigned char *CONVERT_TO_ASCII(unsigned char toConvert, unsigned char *ascii)
{
	//unsigned char ASCII[] = "";
	//unsigned char ascii[3] = "";
	ascii[0] = ((toConvert & 0xF0)>>4);
	ascii[1] = toConvert & 0x0F;
	

	unsigned char i = 0;
	for(i=0; i<2; i++)
	{
		if(ascii[i]<10)
			ascii[i] |= 0x30;
		else
			ascii[i] = 0x40 | (ascii[i]-9);
			
		//ASCII[i] = ascii[i];
	}
	ascii[2] = ' ';
	
	return ascii;	
}

void HANDLE_SENSOR_INPUT(unsigned char input)
{
	//unsigned char input = sensorBuffer->current;
	current = input;
	if(IS_COMMAND(input))
	{
		if(IS_DATA(TOS()))
		{
			//DATA IN TOS
			//uint16_t crc = TOS();
			//crc = crc<<8;
			//crc |= input;
			 
			if(CRC11_CHECK(input, TOS()))
			{
				//PASS
				if(IS_LOG(input))
				{
					LOG();
				}
			}
			else
			{
				//FAIL
				POP();
				REPEAT_REQUEST();				
			}
		}
		else
		{
			//NOT DATA IN TOS
			if(CRC3_CHECK(input))
			{
				//PASS
				if(IS_ACKNOWLEDGE(input))
				{
					POP();
				}
				else
				{
					// NOT ACKNOWLEDGE
					if(IS_REPEAT(input))
					{
						if(TOS() != 0x00)
						{
							unsigned char tos = TOS();
							MESSAGE_SENSOR(&tos);
						}
					}
				}
			}
			else
			{
				//FAIL
				REPEAT_REQUEST();
			}
		}
	}
	else
	{
		//NO COMMAND
		//ADD TO TOS
		PUSH(input);
	}
	//NO
	
	//SENSOR_BUFFER_INIT();
}

void LOG()
{
	*(x++) = TOS();
	POP();
	PUSH(0x40);
	APPLY_CRC3(TOS());
	MESSAGE_SENSOR(&crc3Var);
}

unsigned char IS_LOG(unsigned char packet) {
	packet &= 0b11100000;
	if(packet == 0x20)
		return 1;
	else
		return 0;
}
unsigned char IS_REPEAT(unsigned char packet) {
	packet &= 0b11100000;
	if(packet == 0x60)
		return 1;
	else
		return 0;
}
unsigned char IS_ACKNOWLEDGE(unsigned char packet) {
	packet &= 0b11100000;
	if(packet == 0x40)
		return 1;
	else
		return 0;
}
unsigned char IS_COMMAND(unsigned char packet) {
	return (packet < 0x80);
}
unsigned char IS_DATA(unsigned char packet) {
	return (packet >= 0x80);
}

void REPEAT_REQUEST() {
	APPLY_CRC3(0x60);
	MESSAGE_SENSOR(&crc3Var);
}

unsigned char CRC11_CHECK(unsigned char input, unsigned char tos) {
	return (CRC11(input, tos) == 0x00);
}
unsigned char CRC3_CHECK(unsigned char input) {
	return (CRC3(input) == 0x00);
}



void TRANSMIT(unsigned char buffer)
{
	if(buffer == USER)
	{
		while(userBufferIndex < userBufferSize)
		{
			UDR0 = userBuffer[userBufferIndex++];
			SLEEP();
			//_delay_ms(1000);
			//while((UCSR0A & (1<<UDRE0)) != 1);
		}
	}
	else
	{
		while(sensorBufferIndex < sensorBufferSize)
		{
			UDR1 = sensorBuffer[sensorBufferIndex++];
			SLEEP();
		}
	}
	
}
void MESSAGE_USER(const unsigned char *message)
{
	ADD_TO_BUFFER(USER, message);
	TRANSMIT(USER);
}

void MESSAGE_SENSOR(const unsigned char *message)
{
	ADD_TO_BUFFER(SENSOR, message);
	TRANSMIT(SENSOR);
}

unsigned char TAKE_USER_INPUT()
{
	while(userBufferCurrent != '.')
	SLEEP();
	
	unsigned char userInput = userBuffer[userBufferSize-2];
	
	USER_BUFFER_INIT();
	return userInput;
}
void ADD_TO_BUFFER(unsigned char buffer, const unsigned char *toAdd)
{
	if(buffer == USER)
	{
		strcpy(userBuffer, strcat(userBuffer, toAdd));
		//strcpy(userBuffer, toAdd);
		userBufferSize = strlen(userBuffer);
	}
	else
	{
		strcpy(sensorBuffer, strcat(sensorBuffer, toAdd));
		sensorBufferSize = strlen(sensorBuffer);
		if(toAdd[0] == 0x00)
			sensorBufferSize++;
	}

}

void SLEEP()
{
	sleep_enable(); // arm sleep mode
	sei(); // global interrupt enable
	sleep_cpu(); // put CPU to sleep
}



ISR(TIMER1_COMPA_vect) //iv IVT_ADDR_TIMER1_COMPA //ISR
{
	SENSOR_INIT();
}
// Sample Interrupt handler for USART0 (User side) when a TX is done
ISR(USART0_TX_vect)
{
	sleep_disable();
	
	//CHANGE TO 0 if going --;
	if (userBufferIndex == userBufferSize)
	{ // transmit buffer is emptied
		USER_BUFFER_INIT(); // reinitialize the buffer for next time
		//UCSR0B &= ~((1 << TXEN0) | (1 << TXCIE0)); // disableinterrupt
	}
}

// Sample Interrupt handler for USART0 (User side) when an RX is done
ISR(USART0_RX_vect)
{
	sleep_disable();
	
	while (! (UCSR0A & (1<<RXC0))){}; // Double checking	flag
	
	userBufferCurrent = UDR0;
	ADD_TO_BUFFER(USER, &userBufferCurrent);
	
	if(userBufferCurrent == '.' && isStarted)
	HANDLE_MENU_INPUT();
	
}


//SENSOR ISR
ISR(USART1_TX_vect)
{
	sleep_disable();
	
	if (sensorBufferIndex == sensorBufferSize)
		SENSOR_BUFFER_INIT();
}
ISR(USART1_RX_vect)
{
	sleep_disable();
	
	while (! (UCSR1A & (1<<RXC1))){}; // Double checking	flag
	
	//sensorBuffer->current = UDR0;
	//ADD_TO._BUFFER(sensorBuffer, UDR0);
	
	HANDLE_SENSOR_INPUT(UDR1);
}



void PUSH(unsigned char val)
{	
	if(STACK== (unsigned char *) 0x10CF)
		STACK--;
	*(STACK++) = val;
}
unsigned char POP() {
	if(STACK >= (unsigned char *) 0x1100-STACK_SIZE)
		return *(--STACK);
	return 0x00;
}
unsigned char TOS() {
	if(STACK >= (unsigned char *) 0x1100-STACK_SIZE)
		return *(STACK-1);
	return 0x00;
}