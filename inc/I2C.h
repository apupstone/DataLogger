#ifndef I2C_H
#define I2C_H

#include "stm32f10x.h"

extern "C" { void I2C1_EV_IRQHandler(); void I2C1_ER_IRQHandler(); }

class I2CMaster
{
	public:
		I2CMaster(I2C_TypeDef* periph, GPIO_TypeDef* port_scl, uint32_t pin_scl, GPIO_TypeDef* port_sda, uint32_t pin_sda);
		void tx_data(uint8_t slave_addr, uint8_t mem_addr, uint8_t* val, uint32_t len);
		void rx_data(uint8_t slave_addr, uint8_t mem_addr, uint8_t* val, uint32_t len);
	
	private:
		I2C_TypeDef* m_periph;
		GPIO_TypeDef* m_port_scl;
		uint32_t m_pin_scl;
		GPIO_TypeDef* m_port_sda;
		uint32_t m_pin_sda;
		
};

#endif
