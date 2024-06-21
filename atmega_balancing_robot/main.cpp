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

#include "MPU6050.h"
#include "motor_controller.h"
#include "uart0.h"
#include "timer3_controller.h"
#include "adc.h"
#include "buzzer.h"
#include "kalman_filter.h"

// #define DEBUG0

motor_controller dc_controller;
debug_uart0 debug0;
gyro_controller mpu6050;
timer3_controller led_servo;
adc_controller adc;
buzzer_controller buzzer;
kalman_filter kal;

void timer0_init()
{
  // timer0 for motor control
  TCCR0 |= (1 << WGM01);                 // CTC(Clear Timer on Compare match) mode
  TCCR0 |= ((1 << CS02) | (1 << CS01));  // prescaler 256

  // OCR0 = 0.004s * (clk / prescaler) = 0.004s * (16MHz / 256) = 250
  OCR0 = 250;

  TIMSK |= (1 << OCIE0);
}

ISR(TIMER0_COMP_vect)
{
  PORTD |= 0x04;

  mpu6050.get();

  const double dt = 0.004f;

  double gyro_y = mpu6050.g_gyro[1];

  double acc_z = -mpu6050.g_acc[2] - pow(gyro_y * M_PI / 180.0, 2) * 0.1;
  double acc_x = mpu6050.g_acc[0];

  double angle_ax = atan2(acc_x, acc_z) * 180 / 3.141592;

  double z[2] = { angle_ax, gyro_y };
  double x[2];
  kal.update(z, x);

  static const float kp = 0.005f;   // proportional gain
  static const float kd = 0.0004f;  // derivative gain
  static const float ki = 0.000f;   // integral gain

  static float error_sum = 0.0f;
  static float prev_error = 0.0f;

  float error = x[0] - 5.0;                      // for p term
  float error_diff = (error - prev_error) / dt;  // for d term
  error_sum += error * dt;                       // for i term
  if (error_sum > 20.f)
    error_sum = 100.f;  // anti-windup

  float semi_duty_ratio = -(kp * error + kd * error_diff + ki * error_sum);

  float duty_ratio;

  const double min_ratio = 0.52;

  if (semi_duty_ratio > 0)
  {
    duty_ratio = min_ratio + (1 - min_ratio) * semi_duty_ratio;
  }
  else
  {
    duty_ratio = -min_ratio + (1 - min_ratio) * semi_duty_ratio;
  }

  if (duty_ratio > 0.7)
  {
    duty_ratio = 0.7;
  }
  else if (duty_ratio < -0.7)
  {
    duty_ratio = -0.7;
  }

  dc_controller.set_duty(duty_ratio, duty_ratio);

  prev_error = error;

  static int adc_mode = 0;

  switch (adc_mode)
  {
    case 0: {
	    adc.run_adc(adc_controller::IR_L);
      adc_mode = 1;
      break;
    }
    case 1: {
      adc.run_adc(adc_controller::IR_R);
      static int servo1_cnt = 0;
      if (++servo1_cnt > 39)
      {
        servo1_cnt = 0;

        static int servo_pos = 1;
        if (servo_pos)
        {
          if (adc.get_iir_data(adc_controller::IR_L) < 2.5)
          {
            led_servo.set_servo_degree(1, 0);
          }
          else
          {
            led_servo.set_servo_degree(1, 90);
          }
          servo_pos = 0;
        }
        else
        {
          if (adc.get_iir_data(adc_controller::IR_R) < 3.5)
          {
            led_servo.set_servo_degree(1, 180);
          }
          else
          {
            led_servo.set_servo_degree(1, 90);
          }
          servo_pos = 1;
        }
      }
      //debug0.tx(adc.get_iir_data(adc_controller::IR_L));
      //debug0.tx(",");
      //debug0.tx(adc.get_iir_data(adc_controller::IR_R));
      //debug0.tx("\n");
      adc_mode = 2;
      break;
    }
    case 2: {
      adc.run_adc(adc_controller::PSD);
      buzzer.off_buzzer();
      static int before_time = 0;
      before_time++;
      double psd = 3.5 - adc.get_iir_data(adc_controller::PSD);
      if (psd < 3.0 && psd * 10 < before_time)
      {
        before_time = -1;
        buzzer.on_buzzer(660);
      }
      debug0.tx(adc.idx[6]);
      debug0.tx(" ");
      debug0.tx(adc.data[6][0]);
      debug0.tx(" ");
      debug0.tx(adc.data[6][1]);
      debug0.tx(" ");
      debug0.tx(adc.data[6][2]);
      debug0.tx(" ");
      debug0.tx(adc.iir[6][0]);
      debug0.tx(" ");
      debug0.tx(adc.iir[6][1]);
      debug0.tx(" ");
      debug0.tx(adc.iir[6][2]);
      debug0.tx(" ");
      
      //debug0.tx(adc.get_iir_data(adc_controller::PSD));
      debug0.tx("\n");
      adc_mode = 3;
      break;
    }
    case 3: {
      adc.run_adc(adc_controller::CDS);
      static int light_cnt = 0;
      if (++light_cnt > 9)
      {
        light_cnt = 0;

        double lux = adc.get_cds_lux();

        double duty = (150.0 - lux) / 150.0;

        if (duty > 0.0)
        {
          led_servo.set_led_duty(1.0 - duty);
        }
        else
        {
          led_servo.set_led_duty(1.0);
        }
      }
      //debug0.tx(adc.get_iir_data(adc_controller::CDS));
      //debug0.tx("\n");
      adc_mode = 4;
      break;
    }
    case 4: {
	    adc.run_adc(adc_controller::LM35);
	    static int temp_cnt = 0;
	    if (++temp_cnt > 9)
	    {
		    temp_cnt = 0;
		    double temp = adc.get_lm35_temp();

		    if (temp < 0)
		    {
			    temp = 0;
		    }
		    else if (temp > 50.0)
		    {
			    temp = 50.0;
		    }

		    double servo_deg = temp * 180.0 / 50.0;
		    led_servo.set_servo_degree(0, servo_deg);
	    }
      adc_mode = 0;
      break;
    }

    default:
      break;
  }

  PORTD &= ~0x04;
}

int main(void)
{
  dc_controller.reg_init();
  debug0.reg_init();
  mpu6050.reg_init();
  timer0_init();
  led_servo.reg_init();
  adc.reg_init();
  buzzer.reg_init();

  DDRA = 0xff;
  PORTA = 0x00;

  mpu6050.set();

  DDRD |= 0x04;
  PORTD |= 0x04;

  sei();

  while (1)
  {
  }
}
