/*
 * memory_control.c
 * This is a module specifically written to write to memory on the Da Vinci sattelite bitflip payload.
 * It is not meant to be used as a generalized memory control code, as many hardware specific details
 * such as ports are hard-coded for optimized efficiency
 *
 *  Created on: Jun 17, 2022
 *      Author: Nic Barker
 */

# include "memory_control.h"

void delay_us(uint16_t n) {
    TIM10->CNT = 0;
    while(TIM10->CNT < n);
}

GPIO_InitTypeDef GPIO_InitStruct = {0};

void Set_IO_Outputs() {
	GPIO_InitStruct.Pin = IO_0_Pin|IO_1_Pin|IO_2_Pin|IO_3_Pin|IO_4_Pin|IO_5_Pin|IO_6_Pin|IO_7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Set_IO_Inputs() {
	GPIO_InitStruct.Pin = IO_0_Pin|IO_1_Pin|IO_2_Pin|IO_3_Pin|IO_4_Pin|IO_5_Pin|IO_6_Pin|IO_7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Function to calculate hamming distance (the amount of bits difference between two numbers)
inline int hammingDistance(int n1, int n2) {
    int x = n1 ^ n2;
    int setBits = 0;

    while (x > 0) {
        setBits += x & 1;
        x >>= 1;
    }
    return setBits;
}

MemoryController::MemoryController(GPIO_TypeDef *CE_GPIO_Port, uint16_t CE_Pin) {
	_CE_GPIO_Port = CE_GPIO_Port;
	_CE_Pin = CE_Pin;
	prev_state = WRITING;
	en_state = DISABLED;
}

void MemoryController::set_CE(GPIO_TypeDef *CE_GPIO_Port, uint16_t CE_Pin) {
	_CE_GPIO_Port = CE_GPIO_Port;
	_CE_Pin = CE_Pin;
}

void MemoryController::write(uint32_t addr, uint8_t data) {

	// Enable Memory
	if (en_state != ENABLED) {
		enable();
	}

	// Set 21 bit address
	set_addr(addr);

	// Set IO to Outputs
	if (prev_state != WRITING) {
		Set_IO_Outputs();
		prev_state = WRITING;
	}

	// Set 8 bit data
	GPIOA->ODR = (GPIOA->ODR & 0b11111111111111111111111100000000) | data;

	// Disable Output and Enable Write
	WRITE_EN_GPIO_Port->ODR = (WRITE_EN_GPIO_Port->ODR |= OUT_EN_Pin) &= ~WRITE_EN_Pin; // Set OUT_EN_Pin and reset WRITE_EN_Pin

	// Disable Write
	WRITE_EN_GPIO_Port->ODR |= WRITE_EN_Pin;	// Set WRITE_EN_Pin

}

uint8_t MemoryController::read(uint32_t addr) {

	// Enable Memory
	if (en_state != ENABLED) {
		enable();
	}

	// Disable Write and Enable Output
	WRITE_EN_GPIO_Port->ODR = (WRITE_EN_GPIO_Port->ODR |= WRITE_EN_Pin) &= ~OUT_EN_Pin;	// Set WRITE_EN_Pin and reset OUT_EN_Pin

	// Set 21 bit address
	set_addr(addr);

	// Set IO to Inputs
	if (prev_state != READING) {
		Set_IO_Inputs();
		prev_state = READING;
	}

	// Read IO
	uint8_t data = GPIOA->IDR & 0xff;

	// Disable Output
	OUT_EN_GPIO_Port->ODR = OUT_EN_GPIO_Port->ODR |= OUT_EN_Pin;	// Set OUT_EN_Pin

	return data;
}

inline void MemoryController::set_addr(uint32_t addr) {
	GPIOC->ODR = (GPIOC->ODR & 0b11111111111111110000000000000000) | (addr & 0b1111111111111111);
	GPIOB->ODR = (GPIOB->ODR & 0b11111111111111111111111111001000) | (((addr & 0b1110000000000000000) >> 16) | ((addr & 0b110000000000000000000) >> 15));
}

void MemoryController::clear() {
	// Clear all 2,097,152 addresses
	flood(0);
}

void MemoryController::flood(uint8_t data) {
	// Flood all 2,097,152 addresses with data. At 32MHz this takes 2.2 seconds.

	// Enable Memory
	if (en_state != ENABLED) {
		enable();
	}

	// Set IO to Outputs
	if (prev_state != WRITING) {
		Set_IO_Outputs();
		prev_state = WRITING;
	}

	// Set 8 bit data
	GPIOA->ODR = (GPIOA->ODR & 0xff00) | data;

	// Disable Output and Enable Write
	WRITE_EN_GPIO_Port->ODR = (WRITE_EN_GPIO_Port->ODR |= OUT_EN_Pin) &= ~WRITE_EN_Pin; // Set OUT_EN_Pin and reset WRITE_EN_Pin

	// This is just a fast version of the set_addr function that iterates through all 2,097,152 memory addresses
	uint32_t addr = 0;
	for (uint8_t addr_coarse = 0; addr_coarse < 32; addr_coarse++) {
		GPIOB->ODR = (GPIOB->ODR & 0b11111111111111111111111111001000) | (((addr & 0b1110000000000000000) >> 16) | ((addr & 0b110000000000000000000) >> 15));
		for (uint32_t addr_fine = 0; addr_fine < 65536; addr_fine++) {
			GPIOC->ODR = (GPIOC->ODR & 0b11111111111111110000000000000000) | (addr & 0b1111111111111111);
			addr++;
		}
	}

	// Disable Write
	WRITE_EN_GPIO_Port->ODR |= WRITE_EN_Pin;	// Set WRITE_EN_Pin
}

uint32_t MemoryController::check_for(uint8_t data) {
	// Check entire chip for bytes that are not equal to data. Returns number of incorrect bits. At 32MHz this takes 3.5 seconds.
	// Enable Memory
	if (en_state != ENABLED) {
		enable();
	}

	// Disable Write and Enable Output
	WRITE_EN_GPIO_Port->ODR = (WRITE_EN_GPIO_Port->ODR |= WRITE_EN_Pin) &= ~OUT_EN_Pin;	// Set WRITE_EN_Pin and reset OUT_EN_Pin

	// Set IO to Inputs
	if (prev_state != READING) {
		Set_IO_Inputs();
		prev_state = READING;
	}
	uint32_t wrong_count = 0;

	// Read every address and count bit differences [3.5s when all correct, 17s when all incorrect]
	uint32_t addr = 0;
	for (uint8_t addr_coarse = 0; addr_coarse < 32; addr_coarse++) {
		GPIOB->ODR = (GPIOB->ODR & 0b11111111111111111111111111001000) | (((addr & 0b1110000000000000000) >> 16) | ((addr & 0b110000000000000000000) >> 15));
		for (uint32_t addr_fine = 0; addr_fine < 65536; addr_fine++) {
			GPIOC->ODR = (GPIOC->ODR & 0b11111111111111110000000000000000) | (addr & 0b1111111111111111);
			// Read IO
			if ((GPIOA->IDR & 0xff) != data) {
				wrong_count += hammingDistance(GPIOA->IDR & 0xff, data);
//				char print_buf[10+2];
//				sprintf(print_buf, "E: %07ld\n", addr);
//				HAL_UART_Transmit(&huart1, (uint8_t*) print_buf, sizeof(print_buf), 10);
			}
			addr++;
		}
	}

	// Disable Output
	OUT_EN_GPIO_Port->ODR = OUT_EN_GPIO_Port->ODR |= OUT_EN_Pin;	// Set OUT_EN_Pin

	return wrong_count;
}

void MemoryController::enable() {
	// Enable Memory 1
	HAL_GPIO_WritePin(_CE_GPIO_Port, _CE_Pin, GPIO_PIN_RESET);
	en_state = ENABLED;
}

void MemoryController::disable() {
	// Disable Memory 1
	HAL_GPIO_WritePin(_CE_GPIO_Port, _CE_Pin, GPIO_PIN_SET);
	en_state = DISABLED;
}
