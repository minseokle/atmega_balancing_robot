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


#endif /* INCFILE1_H_ */