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

// Delay for 'n' microseconds using TIM10
// Assumes TIM10 is configured so CNT increments every 1 µs.
void delay_us(uint16_t n) {
    TIM10->CNT = 0;              // Reset timer counter to 0
    while (TIM10->CNT < n);      // Wait until counter reaches 'n'
}

// GPIO initialization structure, zero-initialized
GPIO_InitTypeDef GPIO_InitStruct = {0};

// Configure GPIOA pins IO_0–IO_7 as outputs for writing to memory
void Set_IO_Outputs() {
    // Select pins for the 8-bit data bus
    GPIO_InitStruct.Pin = IO_0_Pin | IO_1_Pin | IO_2_Pin | IO_3_Pin |
                          IO_4_Pin | IO_5_Pin | IO_6_Pin | IO_7_Pin;

    // Set mode to push-pull output: actively drives lines high/low
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;

    // No internal pull-up or pull-down resistors
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // Set output speed to maximum for fast edge transitions
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    // Apply the configuration to GPIOA pins
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


// Configure GPIOA pins IO_0–IO_7 as inputs for reading from memory
void Set_IO_Inputs() {
    // Select pins for the 8-bit data bus
    GPIO_InitStruct.Pin = IO_0_Pin | IO_1_Pin | IO_2_Pin | IO_3_Pin |
                          IO_4_Pin | IO_5_Pin | IO_6_Pin | IO_7_Pin;

    // Set mode to input: MCU will read values driven by the memory chip
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

    // No internal pull-up or pull-down resistors, to avoid biasing the memory outputs
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    // Apply the configuration to GPIOA pins
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

// Function to calculate hamming distance (the amount of bits difference between two numbers)
inline int hammingDistance(int n1, int n2) {
    // XOR the two numbers: bits that are different will be '1', same bits will be '0'
    int x = n1 ^ n2;

    // Counter for the number of differing bits
    int setBits = 0;

    // Count how many '1' bits are in the XOR result
    while (x > 0) {
        setBits += x & 1;  // Add 1 if the least significant bit(the rightest bit) is set
        x >>= 1;           // Shift right by 1 to check the next bit
    }

    // Return the total number of differing bits
    return setBits; 
} //example: n1=0b11001100 and n2 = 0b11100100. x=0b00101000, checks LSB with x&1, adds 1 to setBits if LSB is 1, Shifts x right by one (x >>= 1) so the next bit becomes the LSB, repeats until x=0


// Constructor for the MemoryController class
// Takes the GPIO port and pin used for the chip enable (CE) line
MemoryController::MemoryController(GPIO_TypeDef *CE_GPIO_Port, uint16_t CE_Pin) {

    // Store the given CE port pointer in the private member _CE_GPIO_Port
    _CE_GPIO_Port = CE_GPIO_Port;

    // Store the given CE pin number in the private member _CE_Pin
    _CE_Pin = CE_Pin;

    // Initialize previous I/O direction state to WRITING
    // (so the first read will force a mode change to input)
    prev_state = WRITING;

    // Initialize enable state to DISABLED
    // (chip is assumed to start with CE# high / not selected)
    en_state = DISABLED;
}


// Set (or change) the Chip Enable (CE) port and pin for the memory device
void MemoryController::set_CE(GPIO_TypeDef *CE_GPIO_Port, uint16_t CE_Pin) {
    // Update the stored CE port pointer to the new GPIO port
    _CE_GPIO_Port = CE_GPIO_Port;

    // Update the stored CE pin number to the new GPIO pin
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
    // Lower 16 address bits (A0–A15) → GPIOC[15:0]
    GPIOC->ODR = (GPIOC->ODR & 0b11111111111111110000000000000000)
               | (addr & 0b1111111111111111);

    // Upper 5 address bits (A16–A20) are mapped to specific pins on GPIOB
    // Uses bit masks and shifts to insert the right bits without affecting other pins
    GPIOB->ODR = (GPIOB->ODR & 0b11111111111111111111111111001000)
               | (((addr & 0b1110000000000000000) >> 16)
               | ((addr & 0b110000000000000000000) >> 15));
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
