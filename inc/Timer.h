#ifndef TIMER_H
#define TIMER_H

#include "stm32f10x.h"

extern "C" { void TIM2_IRQHandler(); void TIM3_IRQHandler(); void TIM4_IRQHandler(); }

typedef enum
{
	clock_division_1,
	clock_division_2,
	clock_division_4
} TIM_Clock_Division_TypeDef;

class Timer
{
	public:
		Timer(TIM_TypeDef* timer, TIM_Clock_Division_TypeDef clock_division, uint16_t prescaler);
		void enable();
		void disable();
		
	private:
		TIM_TypeDef* m_timer;
	
};

#endif
