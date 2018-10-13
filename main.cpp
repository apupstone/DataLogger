#include "stm32f10x.h"
#include "DigitalOutput.h"
#include "Timer.h"
#include "I2C.h"

DigitalOutput do_pc8(GPIOC, 8, output_mode_50_mhz, gp_push_pull);

int main()
{
	//Timer tim2(TIM2, clock_division_1, 23999);
	//tim2.enable();
	
	I2CMaster rtc(I2C1, GPIOB, 6, GPIOB, 7);
	
	uint8_t temp_val = 0;
	rtc.rx_data(0x68, 0x75, &temp_val, 1);
	
	while(1)
	{
		//do_pc8.set();
		//for (int i = 0; i < 100000; i++);
		//do_pc8.reset();
		//for (int i = 0; i < 100000; i++);
	}
	
}
