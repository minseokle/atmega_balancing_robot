/*
 * timer3_controller.h
 *
 * Created: 2024-06-20 오후 11:06:46
 *  Author: 0311b
 */ 


#ifndef TIMER3_CONTROLLER_H_
#define TIMER3_CONTROLLER_H_

#include <avr/io.h>

class timer3_controller
{
private:
  double led_duty;
  double servo_degree[2];
public:
  timer3_controller();
  ~timer3_controller();

  void reg_init();

  void set_led_duty(double duty);
  void set_servo_degree(int num,double deg);

private:
  void apply();
};




#endif /* TIMER3_CONTROLLER_H_ */