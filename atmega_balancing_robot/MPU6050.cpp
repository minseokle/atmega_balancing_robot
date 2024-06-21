/*
 * MPU6050.cpp
 *
 * Created: 2024-06-20 오후 8:44:14
 *  Author: 0311b
 */ 

#include "MPU6050.h"


void gyro_controller::reg_init(void)
{
	TWCR = (1 << TWEN);
	TWBR = 12;  //	400 kHz
}

unsigned char gyro_controller::twi_read(unsigned char addr)
{
	TWCR=0x00;
	int cnt;
	unsigned char a_data = 0;

	PORTA = 0x01;
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  // Start
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	cnt=0;
	while ((TWSR & 0xF8) != 0x08)// START 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}

	PORTA++;

	TWDR = 0xD0;                        // Addr : 1101000 + W : 0
	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x18)// SLA+W ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	PORTA++;

	TWDR = addr;                        // Resister Address
	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x28)//Data ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	PORTA++;

	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  // Repeat Start
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x10)//Repeat START 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	PORTA++;

	TWDR = 0xD1;                        // Addr : 1101000 + R : 1
	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	cnt=0;
	while ((TWSR & 0xF8) != 0x40)// SLA+R ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	PORTA++;

	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x58)// ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	PORTA++;

	a_data = TWDR;  // get Data

	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);  // STOP

	_delay_us(50);

	return a_data;
}

int gyro_controller::twi_write(unsigned char addr, char data)
{
	int cnt;
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);  // Start

	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x08)// START 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}

	TWDR = 0xD0;                        // Addr : 1101000 + W : 0
	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	cnt=0;
	while ((TWSR & 0xF8) != 0x18)// ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}

	TWDR = addr;                        // Resister Address
	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x28)// ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}

	TWDR = data;                        // Resister Address
	TWCR = (1 << TWINT) | (1 << TWEN);  // 데이터 전송
	cnt = 0;
	while (!(TWCR & (1 << TWINT)))  // TWINT Flag 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}
	
	cnt=0;
	while ((TWSR & 0xF8) != 0x28)// ACK 상태 대기
	{
		cnt++;
		if (cnt > MAX_CNT)
		{
			return 0;
		}
	}

	TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);  // STOP

	_delay_us(50);
	return 1;
}

void gyro_controller::set()
{
	while (!twi_write(0x6B, 0x00));  // Sleep 끔
	while (!twi_write(0x1A, 0x05));  // DLPF 10 Hz
}
void gyro_controller::get()
{
	g_buffer[0] = twi_read(0x3B);  // x축 가속도
	g_buffer[1] = twi_read(0x3C);
	g_buffer[2] = twi_read(0x3D);  // y축 가속도
	g_buffer[3] = twi_read(0x3E);
	g_buffer[4] = twi_read(0x3F);  // z축 가속도
	g_buffer[5] = twi_read(0x40);
	g_buffer[6] = twi_read(0x43);  // x축 각속도
	g_buffer[7] = twi_read(0x44);
	g_buffer[8] = twi_read(0x45);  // y축 각속도
	g_buffer[9] = twi_read(0x46);
	g_buffer[10] = twi_read(0x47);  // z축 각속도
	g_buffer[11] = twi_read(0x48);

	g_acc_raw[0] = (int)((unsigned int)g_buffer[0] << 8 | (unsigned int)g_buffer[1]);
	g_acc_raw[1] = (int)((unsigned int)g_buffer[2] << 8 | (unsigned int)g_buffer[3]);
	g_acc_raw[2] = (int)((unsigned int)g_buffer[4] << 8 | (unsigned int)g_buffer[5]);
	g_gyro_raw[0] = (int)((unsigned int)g_buffer[6] << 8 | (unsigned int)g_buffer[7]);
	g_gyro_raw[1] = (int)((unsigned int)g_buffer[8] << 8 | (unsigned int)g_buffer[9]);
	g_gyro_raw[2] = (int)((unsigned int)g_buffer[10] << 8 | (unsigned int)g_buffer[11]);

	g_acc[0] = ((double)g_acc_raw[0]) / 16384.0 - 0.0176f;
	g_acc[1] = ((double)g_acc_raw[1]) / 16384.0 - 0.0035f;
	g_acc[2] = ((double)g_acc_raw[2]) / 16384.0 - 0.38f;
	g_gyro[0] = ((double)g_gyro_raw[0]) / 131.0 + 5.359f;
	g_gyro[1] = ((double)g_gyro_raw[1]) / 131.0 + 1.5465f;
	g_gyro[2] = ((double)g_gyro_raw[2]) / 131.0 - 0.3385f;
}
