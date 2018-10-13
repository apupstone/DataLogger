#include "stm32f10x.h"
#include "I2C.h"
#include "DigitalOutput.h"

I2CMaster::I2CMaster(I2C_TypeDef* periph, GPIO_TypeDef* port_scl, uint32_t pin_scl, GPIO_TypeDef* port_sda, uint32_t pin_sda)
{
	// I2C1 or I2C2
	m_periph = periph;
	
	// SCL Port/Pin
	m_port_scl = port_scl;
	m_pin_scl = pin_scl;
	
	// SDA Port/Pin
	m_port_sda = port_sda;
	m_pin_sda = pin_sda;
	
	// Enable GPIO port clock(s)
	DigitalOutput::enable_port_clock(port_scl);
	DigitalOutput::enable_port_clock(port_sda);
	
	// Enable I2C clock
	if (periph == I2C1)
	{
		RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	}
	else if (periph == I2C2)
	{
		RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
	}
	
	// Enable I2C interrupts in the NVIC
	NVIC_EnableIRQ(I2C1_EV_IRQn);
	NVIC_EnableIRQ(I2C1_ER_IRQn);
	
	// Configure SCL in 50 MHz output mode, AF open drain
	if (pin_scl <= 7)
	{
		port_scl->CRL &= ~(0x0F << (pin_scl * 4));
		port_scl->CRL |= output_mode_50_mhz << (pin_scl * 4);
		port_scl->CRL |= af_open_drain << (pin_scl * 4 + 2);
	}
	else if (pin_scl >= 8 && pin_scl <= 15)
	{
		port_scl->CRH &= ~(0x0F << ((pin_scl - 8) * 4));
		port_scl->CRH |= output_mode_50_mhz << ((pin_scl - 8) * 4);
		port_scl->CRH |= af_open_drain << ((pin_scl - 8) * 4 + 2);
	}
	
	// Configure SDA in 50 MHz output mode, AF open drain
	if (pin_sda <= 7)
	{
		port_sda->CRL &= ~(0x0F << (pin_sda * 4));
		port_sda->CRL |= output_mode_50_mhz << (pin_sda * 4);
		port_sda->CRL |= af_open_drain << (pin_sda * 4 + 2);
	}
	else if (pin_sda >= 8 && pin_sda <= 15)
	{
		port_sda->CRH &= ~(0x0F << ((pin_sda - 8) * 4));
		port_sda->CRH |= output_mode_50_mhz << ((pin_sda - 8) * 4);
		port_sda->CRH |= af_open_drain << ((pin_sda - 8) * 4 + 2);
	}
	
	// Start with the I2C peripheral disabled
	m_periph->CR1 &= ~I2C_CR1_PE;
	
	// Peripheral input clock frequency must equal fPCLK1 (APB1 clock): TODO read this in directly
	m_periph->CR2 &= ~I2C_CR2_FREQ;
	m_periph->CR2 |= 0x18; // 24 MHz APB1 frequency
	
	// Standard mode (100 kHz): set frequency and rise time
	m_periph->CCR &= ~I2C_CCR_CCR;
	m_periph->CCR |= 0x78; // CCR = 120 for 100 kHz
	m_periph->TRISE &= ~I2C_TRISE_TRISE;
	m_periph->TRISE |= 0x19; // TRISE = 24 + 1 for 1000 ns x 24 MHz
	
	// Enable all interrupts
	m_periph->CR2 |= I2C_CR2_ITBUFEN;
	m_periph->CR2 |= I2C_CR2_ITEVTEN;
	m_periph->CR2 |= I2C_CR2_ITERREN;
	
	// Enable the I2C peripheral
	m_periph->CR1 |= I2C_CR1_PE;
}

/* Write data at val of length len at memory address mem_addr in slave with address slave_addr */
void I2CMaster::tx_data(uint8_t slave_addr, uint8_t mem_addr, uint8_t* val, uint32_t len)
{
	// Wait until the I2C peripheral is enabled
	while(!(m_periph->CR1 & I2C_CR1_PE))
	{
		m_periph->CR1 |= I2C_CR1_PE;
	}
	
	// Send start flag
	m_periph->CR1 |= I2C_CR1_START;
	// Wait for SB flag to go high
 	while (!(m_periph->SR1 & I2C_SR1_SB))
	{
		int i = m_periph->SR2 & I2C_SR2_MSL;
	}
	
	// Write slave address (shifted left with LSB = 0) into the data register
	m_periph->DR = (slave_addr << 1);
	// After this sequence of operations, the master will clock out the slave address
	
	// Wait until byte transfer is complete
	while (!(m_periph->SR1 & I2C_SR1_ADDR));
	// Read SR2
	uint8_t tra = (m_periph->SR2 & I2C_SR2_TRA) >> I2C_SR2_TRA;
	// After this sequence of operations, the master will clear the ADDR bit
	
	m_periph->DR = mem_addr;
	
	// Wait until memory address has transferred
	while (!(m_periph->SR1 & I2C_SR1_BTF));
	
	uint8_t* pData = val;
	
	for (uint32_t byte_num = 0; byte_num < len; byte_num++)
	{
		m_periph->DR = *(pData++);
		// Wait until data byte has transferred
		while (!(m_periph->SR1 & I2C_SR1_BTF));
	}
	
	// Send stop flag
	m_periph->CR1 |= I2C_CR1_STOP;
}
	
/* Read len bytes into val from memory address mem_addr in slave with address slave_addr */
void I2CMaster::rx_data(uint8_t slave_addr, uint8_t mem_addr, uint8_t* val, uint32_t len)
{
	// Wait until the I2C peripheral is enabled
	while(!(m_periph->CR1 & I2C_CR1_PE))
	{
		m_periph->CR1 |= I2C_CR1_PE;
	}
	
	// Send start flag
	m_periph->CR1 |= I2C_CR1_START;
	// Wait for SB flag to go high	
 	while (!(m_periph->SR1 & I2C_SR1_SB));
	uint32_t temp = m_periph->SR2 & I2C_SR2_MSL;
	
	// Write slave address (shifted left with LSB = 0) into the data register
	m_periph->DR = (slave_addr << 1);
	// After this sequence of operations, the master will clock out the slave address
	
	// Wait until byte transfer is complete
	while (!(m_periph->SR1 & I2C_SR1_ADDR));
	// Read SR2
	uint8_t tra = (m_periph->SR2 & I2C_SR2_TRA) >> I2C_SR2_TRA;
	// After this sequence of operations, the master will clear the ADDR bit
	
	m_periph->DR = mem_addr;
	
	// Wait until memory address has transferred
	while (!(m_periph->SR1 & I2C_SR1_BTF));
	
	// Send repeated start flag
	m_periph->CR1 |= I2C_CR1_START;
	
	// Wait for SB flag to go high
 	while (!(m_periph->SR1 & I2C_SR1_SB))
	{
		int i = m_periph->SR2 & I2C_SR2_MSL;
	}
	
	// Write slave address (shifted left with LSB = 1) into the data register
	m_periph->DR = (slave_addr << 1) | 0x01;
	// After this sequence of operations, the master will clock out the slave address
	
	// Wait until byte transfer is complete
	while (!(m_periph->SR1 & I2C_SR1_ADDR));
	// Read SR2
	tra = (m_periph->SR2 & I2C_SR2_TRA) >> I2C_SR2_TRA;
	// After this sequence of operations, the master will clear the ADDR bit
	
	for (uint32_t byte_num = 0; byte_num < len; byte_num++)
	{
		// Wait for RXNE (i.e. for a byte to appear in the data register)
		while(!(m_periph->SR1 & I2C_SR1_RXNE));
		if (byte_num == len - 1)
		{
			// Send stop flag
			m_periph->CR1 |= I2C_CR1_STOP;
		}
		// Read the data register
		*(val++) = m_periph->DR & 0xFF;
	}
}


void I2C1_EV_IRQHandler()
{
	int i = 0;
}


void I2C1_ER_IRQHandler()
{
	int i = 0;
}


