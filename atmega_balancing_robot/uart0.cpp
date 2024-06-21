/*
 * uart0.cpp
 *
 * Created: 2024-06-20 오후 8:40:48
 *  Author: 0311b
 */ 
#include "uart0.h"



debug_uart0::debug_uart0(){}
void debug_uart0::reg_init(){
	UCSR0A=0x00;
	UCSR0B=(1<<TXEN0);	//tx enable
	UCSR0C=0x06;
	UBRR0H=0x00;
	UBRR0L=0x00;
}

void debug_uart0::tx(char c)
{
	while(!(UCSR0A&(1<<UDRE0)));
	UDR0=c;
}
void debug_uart0::tx(char* str)
{
	while ((*str)!=0)
	{
		tx(*(str++));
	}
}
void debug_uart0::tx(int data,int length)
{
	if(data<0)
	{
		data=-data;
		tx('-');
	}
	if (length==0)
	{
		int temp=data;
		int i=0;
		do
		{
			temp=temp/10;
			i++;
		} while (temp);
		length=i;
	}
	char * str=(char*)malloc(length+1);
	for(int i=length-1;i>=0;i--)
	{
		str[i]=(data%10)+48;
		data/=10;
	}
	str[length]=0;
	tx(str);
	free(str);
}
void debug_uart0::tx(double data,int length,int float_len)
{
	if(data<0)
	{
		data=-data;
		tx('-');
	}
	
	int temp_i=data;
	tx(temp_i,length);
	tx('.');
	int data_pow=1;
	for(int i=0;i<float_len;i++)
	{
		data_pow*=10;
	}
	int temp_d=((unsigned int)(data*data_pow))%data_pow;
	tx(temp_d,float_len);
}