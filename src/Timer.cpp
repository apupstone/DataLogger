#include "stm32f10x.h"
#include "Timer.h"
#include "DigitalOutput.h"

extern DigitalOutput do_pc8;

volatile int num_of_toggles = 0;

Timer::Timer(TIM_TypeDef* timer, TIM_Clock_Division_TypeDef clock_division, uint16_t prescaler)
{
	timer->CR1 &= ~(1<<8 | 1<<9);
	timer->CR1 |= clock_division << 8;
	timer->PSC = prescaler;
	timer->ARR = 0;
	m_timer = timer;
	
	if (m_timer == TIM2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	}
	else if (m_timer == TIM3)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	}
	else if (m_timer == TIM4)
	{
		RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	}
}


void Timer::enable()
{
	m_timer->CR1 |= 0x01;
	
	m_timer->DIER |= 0x01;
	
	m_timer->SR &= ~TIM_SR_UIF;
	
	if (m_timer == TIM2)
	{
		NVIC_EnableIRQ(TIM2_IRQn);
	}
	else if (m_timer == TIM3)
	{
		NVIC_EnableIRQ(TIM3_IRQn);
	}
	else if (m_timer == TIM4)
	{
		NVIC_EnableIRQ(TIM4_IRQn);
	}
}


void Timer::disable()
{
	m_timer->CR1 &= ~0x01;
}


void TIM2_IRQHandler()
{
	if (TIM2->SR & TIM_SR_UIF)
	{
		TIM2->SR &= ~TIM_SR_UIF;
		if (num_of_toggles++ > 360)
		{
			do_pc8.toggle();
			num_of_toggles = 0;
		}
	}
}


void TIM3_IRQHandler()
{
	int i = 0;
}


void TIM4_IRQHandler()
{
	int i = 0;
}

