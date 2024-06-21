/*
 * MPU60500.h
 *
 * Created: 2024-06-19 오후 9:50:43
 *  Author: 0311b
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#define MAX_CNT 1000

#define F_CPU 16000000
#include <avr/io.h>
#include <util/delay.h>

class gyro_controller{
	public:
	
	volatile unsigned char g_buffer[12] = {
		0,
	};
	volatile int g_gyro_raw[3] = {
		0,
	};
	volatile int g_acc_raw[3] = {
		0,
	};
	volatile double g_gyro[3] = {
		0,
	};
	volatile double g_acc[3] = {
		0,
	};


	void reg_init(void);
	void set();
	void get();
	private:
	unsigned char twi_read(unsigned char addr);
	int twi_write(unsigned char addr, char data);
	};



#endif /* MPU60500_H_ */