#ifndef DIGITAL_OUTPUT_H
#define DIGITAL_OUTPUT_H

#include "stm32f10x.h"

typedef enum
{
	input_mode,
	output_mode_10_mhz,
	output_mode_2_mhz,
	output_mode_50_mhz
} GPIO_Mode_TypeDef;

typedef enum
{
	analog_mode,
	floating_input,
	input_pupd
} GPIO_Input_Conf_TypeDef;

typedef enum
{
	gp_push_pull,
	gp_open_drain,
	af_push_pull,
	af_open_drain
} GPIO_Output_Conf_TypeDef;

class DigitalOutput
{
	public:
		DigitalOutput(GPIO_TypeDef* port, uint32_t pin, GPIO_Mode_TypeDef mode, GPIO_Output_Conf_TypeDef conf);
	  void set();
	  void reset();
	  void toggle();
	  void set_to(bool value);
		static void enable_port_clock(GPIO_TypeDef* port);
	
	private:
		GPIO_TypeDef* m_port;
		uint32_t m_pin;
	
};

#endif
