/*
 * adc.cpp
 *
 * Created: 2024-06-21 오전 12:18:36
 *  Author: 0311b
 */

#include "adc.h"
#include <math.h>

adc_controller::adc_controller():adc_res{0,},adc_volt{0,}
,a{
{ 0      , 0     },
{-1.6475 , 0.7009},
{ 0      , 0     },
{ 0      , 0     },
{-1.8227 , 0.8372},
{-1.8227 , 0.8372},
{-1.6475 , 0.7009},
{0       , 0     }},
b{
{0      , 0      , 0     },
{0.0134 , 0.0267 , 0.0134},
{0      , 0      , 0     },
{0      , 0      , 0     },
{0.0036 , 0.0072 , 0.0036},
{0.0036 , 0.0072 , 0.0036},
{0.0134 , 0.0267 , 0.0134},
{0      , 0      , 0     }}
{
}

void adc_controller::reg_init()
{
  ADMUX = 0x00;
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}
void adc_controller::run_adc(int num)
{
  ADMUX = num & 0x07;     // channel 1
  ADCSRA |= (1 << ADSC);  /// adc start
  while (!(ADCSRA & (1 << ADIF)))
    ;
  adc_res[num] = ADC;
  adc_volt[num] = adc_res[num] * 5.0 / 1023.0;

  // iir
  idx[num] = (idx[num] + 1) % 3;
  int now_idx = idx[num];

  double* data_ptr = data[num];
  double* iir_ptr = iir[num];
  double* b_ptr = b[num];
  double* a_ptr = a[num];

  data_ptr[now_idx] = adc_volt[num];

  iir_ptr[now_idx]  =  b_ptr[0] * data_ptr[now_idx] + b_ptr[1] * data_ptr[(now_idx + 2) % 3] + b_ptr[2] * data_ptr[(now_idx + 1) % 3]
                                                    - a_ptr[0] * iir_ptr [(now_idx + 2) % 3] - a_ptr[1] * iir_ptr [(now_idx + 1) % 3];
}
double adc_controller::get_volt(int num)
{
  return adc_volt[num];
}
int adc_controller::get_adc(int num)
{
  return adc_res[num];
}
double adc_controller::get_iir_data(int num)
{
  return iir[num][idx[num]];
}

double adc_controller::get_cds_lux()
{
  double volt = get_iir_data(CDS);
  if(volt<0.000001)
  {
	  volt=0.000001;
  }
  const double R9 = 4700;
  double Rcds = R9 * 5 / volt - R9;
  double gamma = 0.8;
  double power = 1 - ((log10(Rcds) - log10(40000)) / gamma);
  double lux = pow(10, power);
  return lux;
}

double adc_controller::get_lm35_temp()
{
  double volt = adc_volt[LM35];
  double origin_volt = volt / 4.0;  // quadruple amplification
  double deg = origin_volt * 100;
  return deg;
}