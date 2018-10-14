#include "stm32f10x.h"
#include "DigitalOutput.h"
#include "Timer.h"
#include "I2C.h"

DigitalOutput do_pc8(GPIOC, 8, output_mode_50_mhz, gp_push_pull);
uint8_t x_accel = 0;

int main()
{
	//Timer tim2(TIM2, clock_division_1, 23999);
	//tim2.enable();
	
	uint8_t temp_val = 0x00;
	
	I2CMaster mpu6050(I2C1, GPIOB, 6, GPIOB, 7);
	
	// Take accelerometer out of sleep mode.
	mpu6050.tx_data(0x68, 0x6B, &temp_val, 1);
	
	while(1)
	{
		mpu6050.rx_data(0x68, 0x3B, &temp_val, 1);
		//do_pc8.set();
		//for (int i = 0; i < 100000; i++);
		//do_pc8.reset();
		//for (int i = 0; i < 100000; i++);
	}
	
}
