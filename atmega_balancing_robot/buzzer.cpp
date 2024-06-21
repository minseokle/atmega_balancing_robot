/*
 * buzzer.cpp
 *
 * Created: 2024-06-21 오전 3:25:17
 *  Author: 0311b
 */ 
#include "buzzer.h"


void buzzer_controller::reg_init(){
	DDRB|=0x80;
	TCCR2=(1<<WGM21)|(1<<CS22)|(1<<CS20);	//ctc mode,prescaler 1024
	OCR2=0;
}
void buzzer_controller::on_buzzer(int hz)
{
	TCCR2|=(1<<COM20);
	OCR2=7812.5/(double)hz;
}
void buzzer_controller::off_buzzer()
{
	TCCR2&=~(1<<COM20);
	OCR2=0;
}