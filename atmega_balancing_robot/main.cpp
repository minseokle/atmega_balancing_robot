/*
 * atmega_balancing_robot.cpp
 *
 * Created: 2024-06-19 오후 6:31:40
 * Author : 0311b
 */

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "MPU60500.h"
#include "motor_controller.h"
#include "uart0.h"

#define DEBUG0

motor_controller dc_controller;
debug_uart0 debug0;

void timer2_init()
{
  TCCR2 = (1 << WGM21) | (1 << CS22);  // ctc mode , prescaler 256
  OCR2 = 250;                          // 100hz
  TIMSK |= (1 << OCIE2);
}
void  timer0_init()
{
	// timer0 for motor control
	TCCR0 |= (1 << WGM01); // CTC(Clear Timer on Compare match) mode
	TCCR0 |= ((1 << CS02)); // prescaler 1024
	// calculate OCR0 for 8ms
	// 8ms = OCR0 * 1 / (clk / prescaler)
	// OCR0 = 0.008s * (clk / prescaler) = 0.008s * (16MHz / 1024) = 125
	OCR0 = 250;
	
  TIMSK |= (1 << OCIE0);
  
}

#define ANGLE_IRQ_PERIOD 0.004f
#define ANGLE_STD_DEV_OF_GYRO 4
#define ANGLE_STD_DEV_OF_ACCEL 3

struct vec3d
{
  double x, y, z;
};

double P[2][2] = { { 0, 0 }, { 0, 0 } };
double K[2] = { 0, 0 };

double Q_angle = 0.01;
double Q_gyroBias = 0.01;
double KFbias = 0;
double KFangle = 0;
double KFangle_output = 0;
double R_measure = 0.001;

double Kalman(double newAngle, double newrate)
{
  double dt = 0.002;
  // 1. project the state ahead
  double KFrate = newrate - KFbias;
  KFangle += dt * KFrate;

  // 2. project the error covariance ahead
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_gyroBias * dt;

  // 1. Compute the Kalman Gain
  float S = P[0][0] + R_measure;

  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  // 2. Update estimate with measurement ZK
  KFangle += K[0] * (newAngle - KFangle);
  KFbias += K[1] * (newAngle - KFangle);

  // 3. Update error covariance
  double P00_tmp = P[0][0];
  double P01_tmp = P[0][1];

  P[0][0] -= K[0] * P00_tmp;
  P[0][1] -= K[0] * P01_tmp;
  P[1][0] -= K[1] * P00_tmp;
  P[1][1] -= K[1] * P01_tmp;

  return KFangle;
}

volatile double angle_gx;
ISR(TIMER2_COMP_vect)
{

  const double dt = 0.002f;

  double acc_z=-g_acc[2]-pow(g_gyro[1]*M_PI/180.0,2)*0.1;
  double acc_x=g_acc[0];
  
  
  
  
  
  double acc_front=pow(acc_z,2)+pow(acc_x,2)-1.0;
  if(acc_front<0)
  {
	  acc_front=0.0;
  }
  else
  {
	  acc_front=sqrt(acc_front);
  }
  
  
  double acc_g=1;
  

  double angle_ax = atan2(g_acc[0], acc_z) * 180 / 3.141592;
  
  double th2=atan2(acc_x,acc_z);
  double th1=atan2(acc_front,acc_g);
  //double angle_ax=(th2-th1)*180/3.141592;

  angle_gx = g_gyro[1] * dt + angle_gx;

  double alpha = 0.96;
  double roll = alpha * angle_gx + (1.000 - alpha) * angle_ax;

  angle_gx = Kalman(roll, g_gyro[1]);

  debug0.tx("angle : ");
  debug0.tx(angle_ax);
  debug0.tx("\n\r");


#ifdef DEBUG0
  debug0.tx("acc: ");

  for (int i = 0; i < 3; i++)
  {
    debug0.tx(g_acc[i]);
    debug0.tx(", ");
  }
  debug0.tx("     gyro: ");
  for (int i = 0; i < 3; i++)
  {
    debug0.tx(g_gyro[i]);
    debug0.tx(", ");
  }
  debug0.tx("\n\r");
#endif
}

ISR(TIMER0_COMP_vect) {

	static const float dt = 0.004f; // 8ms
	static const float kp = 0.008f; // proportional gain
	static const float kd = 0.004f; // derivative gain
	static const float ki = 0.000f; // integral gain

	static float error_sum = 0.0f;
	static float prev_error = 0.0f;

	float error = angle_gx; // for p term
	float error_diff = (error - prev_error) / dt; // for d term
	error_sum += error * dt; // for i term
	if (error_sum > 20.f) error_sum = 100.f; // anti-windup

	float semi_duty_ratio = -(kp * error + kd * error_diff + ki * error_sum);

	
	float duty_ratio;
	
	const double min_ratio=0.55;
	
	if(semi_duty_ratio>0)
	{
		duty_ratio=min_ratio+(1-min_ratio)*semi_duty_ratio;
	}
	else
	{
		duty_ratio=-min_ratio+(1-min_ratio)*semi_duty_ratio;
	}

	if(duty_ratio>0.7)
	{
		duty_ratio=0.7;
	}
	else if(duty_ratio<-0.7)
	{
		duty_ratio=-0.7;
	}

debug0.tx("ratio : ");
debug0.tx(duty_ratio);
debug0.tx("\n\r");
	
	
	
	dc_controller.set_duty(duty_ratio,duty_ratio);

	prev_error = error;

}

int main(void)
{
  dc_controller.reg_init();
  debug0.reg_init();
  set_TWI();
  timer0_init();
  timer2_init();

  DDRA = 0xff;
  PORTA = 0x00;

  set_MPU6050();
  

  sei();

  while (1)
  {
	  get_MPU6050();
  }
}
