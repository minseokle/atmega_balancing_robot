/*
 * buzzer.h
 *
 * Created: 2024-06-21 오전 3:25:04
 *  Author: 0311b
 */ 


#ifndef BUZZER_H_
#define BUZZER_H_

#include <avr/io.h>

class buzzer_controller{
	public:
	void reg_init();
	void on_buzzer(int hz);
	void off_buzzer();
	};



#endif /* BUZZER_H_ */