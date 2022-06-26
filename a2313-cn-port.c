/*
* a2313-cn-port.c
*
* Created: 14.06.2022 13:16:35
* Author : Alexander.Chad
*/
#define F_CPU 8000000UL
#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#define Enc_1 _BV(PD3)
#define Cn_ID 0b01000000 //ID МК
//#define Cn_ID 0b10000000 //ID МК
//#define Cn_ID 0b11000000 //ID МК
volatile int32_t Pos = 0;

void USART_Init( unsigned int baud )
{
	/* Set baud rate */
	UBRRH = (unsigned char)(baud>>8);
	UBRRL = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSRB = (1<<RXEN)|(1<<TXEN);
	/* Set frame format: 8data, 2stop bit */
	UCSRC = (1<<USBS)|(3<<UCSZ0);
}
void INT_Init() //функция инициализации прерываний
{
	MCUCR=(1<<ISC01)|(1<<ISC00);
	GIMSK=(1<<INT0);
	sei(); // Разрешаем глобальные прерывания
}
void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSRA & (1<<UDRE)) );
	/* Put data into buffer, sends the data */
	UDR = data;
}
unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSRA & (1<<RXC)) );
	/* Get and return received data from buffer */
	return UDR;
}
ISR(INT0_vect){ // прерывание по вектору INT0
	if (PIND & Enc_1){
		Pos--;
		}else{
		Pos++;
	}
}
/*
void UART_send_str (const char *str)
{
while(*str)
USART_Transmit(*str++);
}
*/
int main(void)
{
	DDRD=0; //input
	PORTD=0xFF; //pull_up
	DDRB=0xFF; //output
	PORTB=0; //all off
	USART_Init(51); //9600
	INT_Init();
	/*
	uint8_t buff_len = 20;
	char buffer[buff_len];
	*/
	while (1)
	{
		uint8_t r_byte = USART_Receive();
		if ((r_byte&0b11000000)==Cn_ID){
			if(r_byte&0b00100000){
				PORTB=r_byte&0b00011111;
				}else{
				//snprintf(buffer, buff_len, "%d\r\n", Pos);
				//UART_send_str(buffer);
				unsigned char *rep = (unsigned char *)&Pos;
				for (uint8_t i = 0; i<4; i++)
				{
					USART_Transmit(rep[i]);
				}
			}
		}
	}
}

