#ifndef MOTOR_CONTOLLER_H_
#define MOTOR_CONTOLLER_H_

#include <avr/io.h>

#define  ABS(A) (A>0?A:-A)

class motor_controller
{
	public:
	motor_controller();
  
  void reg_init();
  void set_duty(double left_spd,double right_spd);
  void drive();
	protected:
	private:
  double left_spd_;
  double right_spd_;
};

motor_controller::motor_controller():left_spd_(0.0),right_spd_(0.0)
{
	
}

void motor_controller::reg_init()
{
	DDRB|=0x6F;
	TCCR1A=0xA2;	//fast pwm mode
	TCCR1B=0x19;		//
	ICR1=800;
	
	drive();
}

void motor_controller::set_duty(double left_spd,double right_spd)
{
	left_spd_=left_spd;
	right_spd_=right_spd;
	
	drive();
}

void motor_controller::drive()
{
	if(left_spd_>0)
	{
		PORTB=(PORTB&0xFC)|0x02;	
	}
	else
	{
		PORTB=(PORTB&0xFC)|0x01;
	}
	
	if(right_spd_>0)
	{
		PORTB=(PORTB&0xF3)|0x08;
	}
	else
	{
		PORTB=(PORTB&0xF3)|0x04;
	}
	
	double left_duty=ABS(left_spd_);
	double right_duty=ABS(right_spd_);
	
	OCR1A=(int)(800*left_duty                                             );
	OCR1B=(int)(800*right_duty);
}

#endif /* INCFILE1_H_ */