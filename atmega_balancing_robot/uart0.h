/*
 * uart0.h
 *
 * Created: 2024-06-19 오후 10:39:26
 *  Author: 0311b
 */ 


#ifndef UART0_H_
#define UART0_H_

#include <avr/io.h>
#include <stdlib.h>

class debug_uart0
{
public:
debug_uart0();
void reg_init();
void tx(char c);
void tx(char* str);
void tx(int data,int length=0);
void tx(double data,int length=0,int float_len=3);

protected:
private:
};

#endif /* UART0_H_ */