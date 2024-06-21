/*
 * adc.h
 *
 * Created: 2024-06-21 오전 12:18:45
 *  Author: 0311b
 */

#ifndef ADC_H_
#define ADC_H_

#include <avr/io.h>

class adc_controller
{
	public:
  int adc_res[8];
  double adc_volt[8];

  double a[8][2];
  double b[8][3];

  double data[8][3];
  double iir[8][3];

  int idx[8];

public:
  enum
  {
    CDS = 1,
    IR_R = 4,
    IR_L = 5,
    PSD = 6,
    LM35 = 7
  };
  adc_controller();

  void reg_init();
  void run_adc(int num);
  double get_volt(int num);
  double get_iir_data(int num);
  int get_adc(int num);

  double get_cds_lux();
  double get_lm35_temp();
};

#endif /* ADC_H_ */