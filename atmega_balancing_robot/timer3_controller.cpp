/*
 * timer3_controller.cpp
 *
 * Created: 2024-06-20 오후 11:06:20
 *  Author: 0311b
 */
#include "timer3_controller.h"

timer3_controller::timer3_controller() : led_duty(0.0), servo_degree{ 0.0, 0.0 }
{
}

timer3_controller::~timer3_controller()
{
}

void timer3_controller::reg_init()
{
  DDRE|=0x38;
  TCCR3A = (1 << COM3A1) | (1 << COM3B1) | (1 << COM3C1) | (1 << WGM31);  // fast pwm mode
  TCCR3B = (1 << WGM33) | (1 << WGM32) | (1<<CS31);                          // prescaler 8
  ICR3 = 39999;                                                           // period 20ms
  apply();
}
void timer3_controller::set_led_duty(double duty)
{
  led_duty = duty;
  volatile double temp = ICR3 + 1;
  OCR3A = (unsigned int)(duty * temp);
}
void timer3_controller::set_servo_degree(int num, double deg)
{
  servo_degree[num] = deg;
  double width = 0.5 + 2.0 * deg / 180.0;
  volatile double temp = ICR3 + 1;
  if (num)
  {
    OCR3B = temp * width / 20.0;
  }
  else
  {
    OCR3C = temp * width / 20.0;
  }
}

void timer3_controller::apply()
{
  volatile double temp = ICR3 + 1;
  OCR3A = led_duty * temp;

  double width0 = 0.5 + 2.0 * servo_degree[0] / 180.0;
  double width1 = 0.5 + 2.0 * servo_degree[1] / 180.0;

  OCR3B = temp * width0 / 20.0;
  OCR3C = temp * width1 / 20.0;
}