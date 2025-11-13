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


#define TEMP_MAX 0x1B
#define TEMP_MIN 0x03
#define TEMP_MAX_VOLT 4.0
#define TEMP_MIN_VOLT 2.0
#define TEMP_VAL_INDEX 4

#define MOIST_MAX 0x1E
#define MOIST_MIN 0x02
#define MOIST_MAX_VOLT 4.2
#define MOIST_MIN_VOLT 1.8
#define MOIST_VAL_INDEX 14

#define WATER_MAX 0x1A
#define WATER_MIN 0x04
#define WATER_MAX_VOLT 2.8
#define WATER_MIN_VOLT 2.0
#define WATER_VAL_INDEX 20

#define BAT_MAX 0x1F
#define BAT_MIN 0x01
#define BAT_MAX_VOLT 5.0
#define BAT_MIN_VOLT 3.0
#define BAT_VAL_INDEX 30

#define BAT_LOW 3.2
#define VCC 5.0

#define MOTOR_MAX_SPEED 80
#define MOTOR_MIN_SPEED 20

#define CRC_POLY 0xD4

//HERE
void START_INIT();
void PORTS_INIT();
void USART_INIT();
void LCD_INIT();
void VARS_INIT();

void sendCommand(unsigned char command);
void sendData(unsigned char data);
void sendLCD(unsigned char isCommand, unsigned input);

void TRANSMIT(unsigned char message);

void CONFIG_SENSOR_TIMER();
void CONFIG_MOTOR_TIMER();
void RESET_MOTOR_TIMER();
void RESET_SENSOR_TIMER();

void UPDATE_DISPLAY_MESSAGE();
unsigned char *CONVERT_TO_ASCII(unsigned char toConvert, unsigned char *ascii);


void SLEEP();

void START_SENSOR_CONVERSION();
void TEMP_ADC_SENSOR();
void MOIST_ADC_SENSOR();
void WATER_ADC_SENSOR();
void BAT_ADC_SENSOR();
void CONVERSION_COMPLETE();
void SEND_NEXT();
unsigned char CALCULATE_MOTOR_SPEED();
float ADC_VALUE(float volt);


void APPLY_CRC3(unsigned char input);
unsigned char CRC3(unsigned char input);
unsigned char CRC3_CHECK(unsigned char input);
unsigned char CRC11(unsigned char input, unsigned char tos);
unsigned char APPLY_CRC11(unsigned char data, unsigned char command);

void TRANSMIT(unsigned char buffer);


unsigned char isBatLow = 0;

unsigned char totalChars = 0;

unsigned char isStarted = 0;
unsigned char dataSent = 0x00;
unsigned char sensorInteruptCount = 0;


unsigned char displayMessage[32] = "T=0xVV    M=0xVVW=0xVV    B=0xVV";
unsigned char displayBatteryLow[32] = " Change Battery | Immediately | ";
unsigned char sensorData[4] = "";
unsigned char currentSensorConversion = 0;

unsigned char minValue[4] = "";
float maxVoltsValue[4];
float minVoltsValue[4];
float resolution[4];
float batteryLowVal;

unsigned char crc3Var = 'x';
unsigned char acknowledge;
unsigned char currentSentIndex = 0;
unsigned char reset = 0;

int main(void)
{

		START_INIT();
	
	
		//Enable INT1.
		//GICR = 0x80;
		//Enable ISC11 and ISC10. Rising edge on INT 1.
		//EICRA = 0x0C;
		//EIMSK = 0x02;
	
		//sendData('R');
		//sendData('E');
		//sendData('S');
		//lastChar = '\0';
		//sendData('\0');
	

		isStarted = 1;
		//sei();
		while (isStarted)
		{
			SLEEP();
			//CHECK_INPUT();
		}		

}

void START_INIT()
{
	PORTS_INIT();
	LCD_INIT();
	USART_INIT();
	VARS_INIT();
	//RESET_MOTOR_TIMER();
	//RESET_SENSOR_TIMER();
}

void VARS_INIT()
{
	isBatLow = 0;
	totalChars = 0;
	isStarted = 0;
	dataSent = 0x00;
	sensorInteruptCount = 0;

	//displayMessage[32] = "T=0xVV    M=0xVVW=0xVV    B=0xVV";
	//displayBatteryLow[32] = " Change Battery | Immediately | ";
	strcpy(sensorData, "    ");
	currentSensorConversion = 0;
	
	
	maxVoltsValue[0] = ADC_VALUE(TEMP_MAX_VOLT);
	minVoltsValue[0] = ADC_VALUE(TEMP_MIN_VOLT);
	
	maxVoltsValue[1] = ADC_VALUE(MOIST_MAX_VOLT);
	minVoltsValue[1] = ADC_VALUE(MOIST_MIN_VOLT);
	
	maxVoltsValue[2] = ADC_VALUE(WATER_MAX_VOLT);
	minVoltsValue[2] = ADC_VALUE(WATER_MIN_VOLT);
	
	maxVoltsValue[3] = ADC_VALUE(BAT_MAX_VOLT);
	minVoltsValue[3] = ADC_VALUE(BAT_MIN_VOLT);
	
	resolution[0] = ((maxVoltsValue[0]-minVoltsValue[0])/(TEMP_MAX-TEMP_MIN));
	resolution[1] = ((maxVoltsValue[1]-minVoltsValue[1])/(MOIST_MAX-MOIST_MIN));
	resolution[2] = ((maxVoltsValue[2]-minVoltsValue[2])/(WATER_MAX-WATER_MIN));
	resolution[3] = ((maxVoltsValue[3]-minVoltsValue[3])/(BAT_MAX-BAT_MIN));
	
	minValue[0] = TEMP_MIN;
	minValue[1] = MOIST_MIN;
	minValue[2] = WATER_MIN;
	minValue[3] = BAT_MIN;
	
	batteryLowVal = ADC_VALUE(BAT_LOW);
	
	if(!reset)
		acknowledge = 0;
	currentSentIndex = 0;
	reset = 0;
}


void PORTS_INIT()
{
	/*
	DDRA.0 = RS
	DDRA.1 = RW
	DDRA.2 = E
	*/
	DDRA = 0xFF;
	
	//Motor output
	DDRB = 0xFF;
	PORTB = 1;
	
	//LCD DATA OUTPUT
	DDRC = 0xFF;
	
	//SENSOR Input.
	DDRF = 0x00;
}

void USART_INIT()
{
	UCSR0B = (1<<TXEN0 | 1<<TXCIE0 | 1<<RXEN0 | 1<<RXCIE0);
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	UBRR0H = 0x00;
	UBRR0L = 0x33;	
	sei();
}
void LCD_INIT()
{
	totalChars = 0;
	//2 Line display:
	//https://morf.lv/simple-library-for-driving-20x4-lcd-with-4bits	
	sendCommand(LCD_2_LINE_MODE);
	sendCommand(DISPLAY_ON_BLINK);
	sendCommand(CLEAR_DISPLAY);
	sendCommand(SHIFT_AFTER_DISPLAY);
	_delay_ms(10);
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
	sendLCD(1, data);	
	totalChars++;
	
	unsigned char lineNum = totalChars/16;
	unsigned char currentChars = totalChars%16;
	unsigned char remainingChars = 16-currentChars;
	
	if(data == '\r' && currentChars != 0)
	{
		totalChars += remainingChars;
		lineNum++;
	}
		
	if(remainingChars==16 || data == '\r')
	{
		if(lineNum == 1)
			sendCommand(0xC0);
		//else if(lineNum == 2)
			//LCD_INIT();
	}
}

void sendLCD(unsigned char isData, unsigned input)
{
	PORTA = 0x00+isData;
	PORTC = input;
		
	PORTA = 0x04+isData;
	_delay_ms(1);
	PORTA = 0x00+isData;
	_delay_ms(1);
}



ISR(USART0_TX_vect) {
	sleep_disable();
}

// Sample Interrupt handler for USART0 (User side) when an RX is done
ISR(USART0_RX_vect)
{
	sleep_disable();
	
	while (! (UCSR0A & (1<<RXC0))){}; // Double checking	flag
	
	unsigned char commandIn = UDR0;
	
	if(!CRC3_CHECK(commandIn))
	{
		APPLY_CRC3(0x60);
		TRANSMIT(crc3Var);
		return;
	}
	
	commandIn &= 0b11100000;
	
	if(commandIn == 0x00) //RESET
	{
		//START_INIT();
		CONFIG_SENSOR_TIMER();
		CONFIG_MOTOR_TIMER();
		APPLY_CRC3(0x40);
		TRANSMIT(crc3Var);
		reset = 1;
		acknowledge = 1;
	}
	else if(commandIn == 0x40) //Acknowledge
	{
		acknowledge = 1;
		SEND_NEXT();
	}
	else if(commandIn == 0x60)
	{
		acknowledge = 1;
		currentSensorConversion--;
		SEND_NEXT();
	}
	//sendData(UDR0);
}


void SLEEP()
{
	sleep_enable(); // arm sleep mode
	sei(); // global interrupt enable
	sleep_cpu(); // put CPU to sleep
}







void CONFIG_SENSOR_TIMER()
{
	//500 or 1000 or 2000
	//counter value
	TCNT1 = 0;
	TCCR1A = 0x00;
	TCCR1B = 0b00001101; // CTC with 1024 prescalling
	
	// (1/8Mhz)*1024 = 128us
	
	// s/us = *1000000;
	OCR1A = (5*1000000/128);
	
	TIMSK = (1<<OCIE1A);
}


ISR(TIMER1_COMPA_vect) //iv IVT_ADDR_TIMER1_COMPA //ISR
{
	sleep_disable();
		
	sensorInteruptCount++;
	if(sensorInteruptCount%2)
	{
		//TURN OFF MOTOR.
		OCR0 = 0;
		return;
	}

	START_SENSOR_CONVERSION();
}

void UPDATE_DISPLAY_MESSAGE()
{
	unsigned char toASCII[3] = "";
	CONVERT_TO_ASCII(sensorData[0], toASCII);
	displayMessage[TEMP_VAL_INDEX] = toASCII[0];
	displayMessage[TEMP_VAL_INDEX+1] = toASCII[1];
	
	CONVERT_TO_ASCII(sensorData[1], toASCII);
	displayMessage[MOIST_VAL_INDEX] = toASCII[0];
	displayMessage[MOIST_VAL_INDEX+1] = toASCII[1];
	
	CONVERT_TO_ASCII(sensorData[2], toASCII);
	displayMessage[WATER_VAL_INDEX] = toASCII[0];
	displayMessage[WATER_VAL_INDEX+1] = toASCII[1];
	
	CONVERT_TO_ASCII(sensorData[3], toASCII);
	displayMessage[BAT_VAL_INDEX] = toASCII[0];
	displayMessage[BAT_VAL_INDEX+1] = toASCII[1];
}

void CONFIG_MOTOR_TIMER()
{
	//500 or 1000 or 2000
	//counter value
	TCNT0 = 0;
	TCCR0 = 0b01100001;		//PWM Phase correct, non-inverting.
	//TCCR3B = 0b00001101; // CTC with 1024 prescalling
	
	// (1/8Mhz)*1024 = 128us
	
	// 5 seconds.
	// s/us = *1000000;
	OCR0 = 0;
	
	//TIMSK |= (1<<OCIE3A);
}


void RESET_MOTOR_TIMER()
{
	TCNT3 = 0;
	TCCR3A = 0x00;
	TCCR3B = 0x00;
	OCR3A = 0x00;
	TIMSK &= ~(1<<OCIE3A);
}
void RESET_SENSOR_TIMER()
{
	TCNT1 = 0;
	TCCR1A = 0x00;
	TCCR1B = 0x00;
	OCR1A = 0x00;
	TIMSK &= ~(1<<OCIE1A);
}

/*
ISR(TIMER0_COMPA_vect) //iv IVT_ADDR_TIMER1_COMPA //ISR
{
	sleep_disable();
	//RESET_MOTOR_TIMER();
	
	//TODO: DISABLE MOTOR.
}
*/

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




void START_SENSOR_CONVERSION()
{
	//Enable ADC and clk 128. and interupt.
	//ADCSRA = 0x87;
	ADCSRA = 0x8F;
	//Right adjusted and internal 2.56 ref.
	//ADMUX = 0b11000000;

	
	//ADCSRA |= (1<<ADSC); // start conversion
	
	//while ((ADCSRA & (1<<ADIF))==0);
	// wait for end of conversion
	//ADCSRA |= (1<<ADIF);

	//sensorData[0] = ADCH;
	
	//CONVERSION_COMPLETE();
	TEMP_ADC_SENSOR();
}

ISR(ADC_vect)
{
	sleep_disable();
	
	unsigned char currentADC = ADCH;
	float result = ((currentADC-minVoltsValue[currentSensorConversion])/resolution[currentSensorConversion])+minValue[currentSensorConversion]+1;
	
	sensorData[currentSensorConversion] = result;
	
	if(currentSensorConversion==3)
		if(currentADC<=batteryLowVal)
			isBatLow = 1;
		
	currentSensorConversion++;
	
	ADC = 0x0000;
	ADCSRA |= (1<<ADIF);
	ADCSRA &= ~(1<<ADSC);
	//currentSensorConversion++;
	
	if(currentSensorConversion==1)
		MOIST_ADC_SENSOR();
	else if(currentSensorConversion==2)
		WATER_ADC_SENSOR();
	else if(currentSensorConversion==3)
		BAT_ADC_SENSOR();
	else
		CONVERSION_COMPLETE();
	
}

void SEND_NEXT()
{
	//No acknowledge recieved or 
	if(currentSentIndex >= 4 || acknowledge == 0)
		return;
	
	unsigned char dataPacket = sensorData[currentSentIndex];
	dataPacket |= currentSentIndex<<5;
	dataPacket |= 1<<7;
	
	printf("%d 's datapacket = %d\n", currentSentIndex, dataPacket);
	
	unsigned char command = APPLY_CRC11(dataPacket, 0b00100000);
	//TODO COMMAND
	TRANSMIT(dataPacket);
	_delay_ms(1);
	TRANSMIT(command);
	
	currentSentIndex++;
	acknowledge = 0;
}

void CONVERSION_COMPLETE()
{
	ADCSRA = 0x00;
	currentSensorConversion = 0;
	currentSentIndex = 0;
	
	//CONFIG_MOTOR_TIMER();
	
	//TODO
	//START MOTOR..
	unsigned char motorSpeed = CALCULATE_MOTOR_SPEED();
	OCR0 = motorSpeed;
	
	
	UPDATE_DISPLAY_MESSAGE();
	
	unsigned char i = 0;
	
	LCD_INIT();
	
	for(i=0; i<32; i++)
	{
		if(isBatLow)
			sendData(displayBatteryLow[i]);
		else
			sendData(displayMessage[i]);
	}
	
	//TRANSMIT DATA.
	SEND_NEXT();
}


//TODO ALL THE SENSORS
void TEMP_ADC_SENSOR()
{
	// 2.56 V Vref, ADC0 single ended  data will be left-justified
	ADMUX = 0x60;
	//ADMUX |= (1<<MUX0);	
	//Start Conversion
	ADCSRA |= (1<<ADSC);	
	
	//return 0x00;
}
void MOIST_ADC_SENSOR()
{
	ADMUX = 0x60;
	//ADC 1 selected.
	ADMUX |= (1<<MUX0);
	//Start Conversion
	ADCSRA |= (1<<ADSC);
	
	//return 0x00;
}
void WATER_ADC_SENSOR()
{
	ADMUX = 0x60;
	//ADC 2 selected.
	ADMUX |= (1<<MUX1);
	//Start Conversion
	ADCSRA |= (1<<ADSC);
	
	
	//return 0x00;
}
void BAT_ADC_SENSOR()
{
	ADMUX = 0x60;
	//ADC 3 selected.
	ADMUX |= (1<<MUX0);
	ADMUX |= (1<<MUX1);
	//Start Conversion
	ADCSRA |= (1<<ADSC);
	//TODO CHECK IF BAT IS LOW.
	
	
	//return 0x00;
}

unsigned char CALCULATE_MOTOR_SPEED()
{
	float speed = (((sensorData[1]-MOIST_MIN)/(MOIST_MAX-MOIST_MIN))*(MOTOR_MAX_SPEED-MOTOR_MIN_SPEED))+MOTOR_MIN_SPEED;
	unsigned char OCRValue = ((speed*255)/100);
	return ((unsigned char) OCRValue);
}

/*
unsigned char NORMALIZE_VALUE(unsigned char value, unsigned char maxVolt, unsigned char minVolt, unsigned char maxVal, unsigned char minVal)
{
	unsigned char steps = maxVal-minVal;
	
	unsigned char minVoltValue = ADC_VALUE(minVolt);
	
	float resolution = (ADC_VALUE(maxVolt)-minVoltValue)/steps;
	
	float result = ((value-minVoltValue)/resolution)+minVal;
	
}
*/
float ADC_VALUE(float volt)
{
	
	//if(((int) volt) == ((int) VCC))
	//	return 255.0;
	
	return (((volt*1024)/VCC)/4);
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
unsigned char CRC3_CHECK(unsigned char input) {
	return (CRC3(input) == 0x00);
}

unsigned char current_crc = -1;

unsigned char CRC11(unsigned char data, unsigned char command)
{
	unsigned char crc = data;
	unsigned char packet = command;
	current_crc = crc;
	int counter;
	for(counter = 0; counter<11; counter++)
	{
		if(crc >= 0x80)
			crc = crc ^ CRC_POLY;
		
		crc = (crc<<1);
		unsigned char temp = (0b10000000 & packet);
		packet = packet<<1;
		crc |= (temp>>7);
	}
	crc = crc>>3;
	current_crc = crc;
	return crc;
}

//REturns command with correct CRC11.
unsigned char APPLY_CRC11(unsigned char data, unsigned char command)
{
	unsigned char crc = CRC11(data, command);
	command &= 0b11100000;
	command |= crc;
	
	return command;
}