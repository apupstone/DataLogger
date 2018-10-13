#include "DigitalOutput.h"


void DigitalOutput::enable_port_clock(GPIO_TypeDef* port)
{
	if (port == GPIOA)
	{
	  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	}
	else if (port == GPIOB)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	}
	else if (port == GPIOC)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
	}
	else if (port == GPIOD)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPDEN;
	}
	else if (port == GPIOE)
	{
		RCC->APB2ENR |= RCC_APB2ENR_IOPEEN;
	}
}
	


DigitalOutput::DigitalOutput(GPIO_TypeDef* port, uint32_t pin, GPIO_Mode_TypeDef mode, GPIO_Output_Conf_TypeDef conf)
{
	// Enable clock
	enable_port_clock(port);
	
	m_port = port;
	m_pin = pin;
	
	if (m_pin <= 7)
	{
		m_port->CRL &= ~(0x0F << (m_pin * 4));
		m_port->CRL |= mode << (m_pin * 4);
		m_port->CRL |= conf << (m_pin * 4 + 2);
	}
	else if (m_pin >= 8 && m_pin <= 15)
	{
		m_port->CRH &= ~(0x0F << ((m_pin - 8) * 4));
		m_port->CRH |= mode << ((m_pin - 8) * 4);
		m_port->CRH |= conf << ((m_pin - 8) * 4 + 2);
	}
}


void DigitalOutput::set()
{
	m_port->BSRR = 1 << m_pin;
}


void DigitalOutput::reset()
{
	m_port->BSRR = 1 << (m_pin + 16);
}


void DigitalOutput::toggle()
{
	if (((m_port->ODR >> m_pin) & 0x01) == 1)
	{
		m_port->ODR &= ~(1 << m_pin);
	}
	else
	{
		m_port->ODR |= (1 << m_pin);
	}
}


void DigitalOutput::set_to(bool value)
{
	if (value)
	{
		m_port->ODR |= (1 << m_pin);
	}
	else
	{
		m_port->ODR &= ~(1 << m_pin);
	}
}

