// ----------------------------------------------------------------------------
// School: University of Victoria, Canada.									//
// Course: ECE 355 "Microprocessor-Based Systems".							//
//																			//
// Name: Philippe Larocque V00903595										//
// Name: Harman Sangha V00883210											//
//																			//
// See "system/include/cmsis/stm32f0xx.h" for register/bit definitions.		//
// See "system/src/cmsis/vectors_stm32f0xx.c" for handler declarations.		//
// ----------------------------------------------------------------------------

#include <stdio.h>
#include "diag/Trace.h"
#include "cmsis/cmsis_device.h"


// STM32F0 empty sample (trace via $(trace)).

// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the $(trace) output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).


// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"


/* Clock prescaler for TIM2 timer: no prescaling */
#define myTIM2_PRESCALER ((uint16_t)0x0000)
/* Maximum possible setting for overflow */
#define myTIM2_PERIOD ((uint32_t)0xFFFFFFFF)


// Function Declarations

void myGPIOA_Init(void);	// A pins setup
void myTIM2_Init(void); 	// Timer 2
void myEXTI_Init(void); 	// Trigger

void myADC_Init(void); 		// ADC
void myDAC_Init(void); 		// DAC

void myGPIOB_Init(void); 							// B pins setup
void mySPI_Init(void); 								// SPI
void LCD_send(uint8_t command, uint8_t data); 		// LCD
void LCD_send_nib(uint8_t command, uint8_t data); 	// LCD
void LCD_UpdateFreq(float freq); 					// LCD Frequency
void LCD_UpdateResistance(float ohm); 				// LCD Resistance

void myTIM3_Init(); 				// Delay
void tempDelay(uint32_t msec); 	// Delay


// Global Variables

int delay = 20;   		// 5ms delay
int update = 0;			// Update counter for LCD
float calc_freq;		// Frequency measured
float potentiometer;	// Resistance measured
uint32_t adc_val;		// ADC read value
uint32_t dac_val;		// DAC write value

float min_vout = 1.0;	// Minimum voltage
float max_vout = 3.0;	// Maximum voltage

int main(int argc, char* argv[])
{

	trace_printf("This is ECE 355 Final Project...\n");
	trace_printf("System clock: %u Hz\n", SystemCoreClock);

	myGPIOA_Init();		// A0 = pot, A1 = timer output, A4 = opto input
	myTIM2_Init();		/* Initialize timer TIM2 */
	myEXTI_Init();		/* Initialize EXTI */

	myADC_Init();
	myDAC_Init();

	myGPIOB_Init();		// B3 = data (17), B5 = clock (21), B7 = latch (25)
	mySPI_Init();
	myTIM3_Init();		// extra timer, tim3

	// Set up LCD
	tempDelay(delay);
	LCD_send_nib(0x0, 0x3);	tempDelay(delay);		// 8 bit x 1
	LCD_send_nib(0x0, 0x3);	tempDelay(delay);		// 8 bit x 2
	LCD_send_nib(0x0, 0x3);	tempDelay(delay);		// 8 bit x 3
	LCD_send_nib(0x0, 0x2);	tempDelay(delay);		// 4 bit
	LCD_send(0x0, 0x28); tempDelay(delay);			// 4 bit mode with 2 lines
	LCD_send(0x0, 0x0C); tempDelay(delay);			// 0000 1100, configure mode, display on, cursor off, cursor blink off
	LCD_send(0x0, 0x01); tempDelay(delay);			// clear screen
	LCD_send(0x0, 0x06); tempDelay(delay);			// 0000 0110, data entry mode, cursor movement, display shift off

	while (1)
	{
		adc_val = ADC1->DR; // Read ADC value from pot
		potentiometer = 5000.0*adc_val/0xFFF; // Convert to Ohms
		dac_val = ((float)((float)adc_val/0xFFF)*(max_vout-min_vout)+min_vout)*(0xFFF/max_vout);     // limit to only 1V or higher (mapping)
		DAC->DHR12R1 = dac_val; // Output to DAC

		//rewrite below
		
		//update++; 									// For LCD updates
		//tempDelay(delay);							// For freeing up proccessor time

		//if(update >= 20)
		//{
		//	NVIC_DisableIRQ(EXTI0_1_IRQn);			// Disable signal measurement interrupt
		//	LCD_UpdateFreq(calc_freq);				// Update the frequency line on LCD
		//	LCD_UpdateResistance(potentiometer);	// Update the resistance line on LCD
		//	update = 0;								// Reset update counter
		//	NVIC_EnableIRQ(EXTI0_1_IRQn);			// Enable signal measurement interrupt
		//}
		
		tempDelay(500);
		
		LCD_UpdateFreq(calc_freq);				// Update the frequency
		LCD_UpdateResistance(potentiometer);	// Update the resistance
		
	}
	
	return 0;

}

void myGPIOA_Init()
{
	/* Enable clock for GPIOA peripheral */
	// Relevant register: RCC->AHBENR
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	/* Configure PA1 as input for the signal*/
	// Relevant register: GPIOA->MODER
	GPIOA->MODER &= ~GPIO_MODER_MODER1;
	/* Ensure no pull-up/pull-down for PA1 */
	// Relevant register: GPIOA->PUPDR
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR1;

	// PA0 = ADC input
	GPIOA->MODER |= GPIO_MODER_MODER0;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR0;

	// PA4 = DAC output
	GPIOA->MODER |= GPIO_MODER_MODER4;
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR4;

}

void myGPIOB_Init()
{
	// Enable B pins
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	
	// PB7 = output LCD Latch
	GPIOB->MODER |= GPIO_MODER_MODER7_0;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR7_0;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR7; // fast

	// PB3 = output LCD Clock
	GPIOB->MODER |= GPIO_MODER_MODER3_1;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR5;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR3; // fast

	// PB5 = output LCD Data
	GPIOB->MODER |= GPIO_MODER_MODER5_1;
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR5;
	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEEDR5; // fast

}

void myADC_Init()
{
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN; // Enable ADC clock

    ADC1->CR = ADC_CR_ADCAL; // Calibrate ADC
    while (ADC1->CR == ADC_CR_ADCAL) {};

    ADC1->CFGR1 |= (ADC_CFGR1_CONT | ADC_CFGR1_OVRMOD); // ADC continuous and overwrite

    ADC1->CHSELR = ADC_CHSELR_CHSEL0; // configure PA0 as ADC input

    ADC1->CR |= ADC_CR_ADEN; // enable ADC
    while (!(ADC1->ISR & ADC_ISR_ADRDY)) {};

    ADC1->CR |= ADC_CR_ADSTART; // start

}

void myDAC_Init()
{
	RCC->APB1ENR |= RCC_APB1ENR_DACEN; // enable DAC clock

	DAC->CR |= DAC_CR_EN1; // enable DAC
}

void mySPI_Init()
{
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // enable SPI clock

	SPI_InitTypeDef SPI1_initStruct; // Create a structure
	SPI_StructInit(&SPI1_initStruct); // Fill structure with defaults

	SPI1_initStruct.SPI_Direction = SPI_Direction_1Line_Tx; // Transmitting only
	SPI1_initStruct.SPI_Mode = SPI_Mode_Master; // SPI master
	SPI1_initStruct.SPI_NSS = SPI_NSS_Soft; // Only one device, chip select not required
	SPI1_initStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256; // set SPI frequency

	SPI_Init(SPI1, &SPI1_initStruct); // Use the above settings to configure the SPI1 bus

	SPI_Cmd(SPI1, ENABLE); // start SPI
}

void myTIM3_Init()
{
	/* Enable clock for TIM3 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	/* Configure TIM3: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM3->CR1
	TIM3->CR1 |= 0x008C;

	/* Set clock prescaler value */
	TIM3->PSC = 48000;
	/* Set auto-reloaded delay */
	TIM3->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM3->EGR
	TIM3->EGR |= 0x0001;
}

void tempDelay(uint32_t msec)
{
	TIM3->CNT |= 0;						// clear timer
	TIM3->ARR = msec;  					// set time
	TIM3->EGR |= 0x1; 					// Update timer register
	TIM3->CR1 |= TIM_CR1_CEN;			// Start timer
	while(!(TIM3->SR & TIM_SR_UIF));	// Wait for timer to count
	TIM3->CR1 &= ~(TIM_CR1_CEN);		// Stop timer
	TIM3->SR &= ~(TIM_SR_UIF);			// Clear interrupt flag

}

void LCD_send(uint8_t command, uint8_t data)				// LCD send 8 bits of data function
{

	uint8_t high = data>>4;
	uint8_t low = data & 0x0F;
	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	SPI_SendData8(SPI1, (0x00 | command | data>>4) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x8) == RESET));	// Wait for SPI to finish current operations and be free
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	SPI_SendData8(SPI1, (0x80 | command | data>>4) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x8) == RESET));	// Wait for SPI to finish current operations and be free
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	SPI_SendData8(SPI1, (0x00 | command | data>>4) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x8) == RESET));	// Wait for SPI to finish current operations and be free
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	SPI_SendData8(SPI1, (0x00 | command | (data&0x0F)) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	SPI_SendData8(SPI1, (0x80 | command | (data&0x0F)) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	SPI_SendData8(SPI1, (0x00 | command | (data&0x0F)) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));	// Wait for SPI to finish current operations and be free
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

}

void LCD_send_nib(uint8_t command, uint8_t data)			// LCD send 4 bits of data function
{

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));
	SPI_SendData8(SPI1, (0x0 | command | data) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));
	SPI_SendData8(SPI1, (0x80 | command | data) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

	GPIOB->BRR = 0x80;										// Set Latch pin to LOW to update the shift register with new value
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));
	SPI_SendData8(SPI1, (0x0 | command | data) );
	while(!(SPI_I2S_GetFlagStatus(SPI1, 0x80) == RESET));
	GPIOB->BSRR = 0x80;										// Set Latch pin to High to output the shift register new value

}

void LCD_UpdateFreq(float freq)
{
	char LCD_freq[8];										// Create char array for line 1
	sprintf(LCD_freq, "F:%04dHz", (int)freq);				// Fill array with the line frequency formating

	LCD_send(0x0, (0x80 | 0x0 | 0) );						// Move the cursor to row 1 and col 0 -> (command, cursor move | row | col)
	tempDelay(delay);

	for(int x=0 ; x<8 ; x++){
		tempDelay(1);
		LCD_send(0x40, LCD_freq[x]);						// Output each char to the LCD
	}

}

void LCD_UpdateResistance(float ohm)
{
	char LCD_ohms[7];										// Create char array for line 2
	sprintf(LCD_ohms, "R:%04d", (int)ohm);					// Fill array with the line resistance formating
	LCD_ohms[6] = 0xF4;										// Set last char as Ohm symbol from ASCII table

	LCD_send(0x0, (0x80 | 0x40 | 0) );						//Move the cursor to row 2 and col 0 -> (command, cursor move | row | col)
	tempDelay(delay);

	for(int x=0 ; x<7 ; x++){
		tempDelay(1);
		LCD_send(0x40, LCD_ohms[x]);						// Output each char to the LCD
	}

}

void myTIM2_Init()
{
	/* Enable clock for TIM2 peripheral */
	// Relevant register: RCC->APB1ENR
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	/* Configure TIM2: buffer auto-reload, count up, stop on overflow,
	 * enable update events, interrupt on overflow only */
	// Relevant register: TIM2->CR1
	TIM2->CR1 |= 0x008C;

	/* Set clock prescaler value */
	TIM2->PSC = myTIM2_PRESCALER;
	/* Set auto-reloaded delay */
	TIM2->ARR = myTIM2_PERIOD;

	/* Update timer registers */
	// Relevant register: TIM2->EGR
	TIM2->EGR |= 0x0001;

	/* Assign TIM2 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[3], or use NVIC_SetPriority
	NVIC_SetPriority(TIM2_IRQn, 0);						// Set the priority

	/* Enable TIM2 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(TIM2_IRQn);							// Enable the interrupt on timer

	/* Enable update interrupt generation */
	// Relevant register: TIM2->DIER
	TIM2->DIER |= TIM_DIER_UIE;							// Update interrupts?
}

void myEXTI_Init()
{
	/* Map EXTI1 line to PA1 */
	// Relevant register: SYSCFG->EXTICR[0]
	SYSCFG->EXTICR[0] = SYSCFG_EXTICR1_EXTI1_PA;

	/* EXTI1 line interrupts: set rising-edge trigger */
	// Relevant register: EXTI->RTSR
	EXTI->RTSR |= EXTI_RTSR_TR1;

	/* Unmask interrupts from EXTI1 line */
	// Relevant register: EXTI->IMR
	EXTI->IMR |= EXTI_IMR_MR1;

	/* Assign EXTI1 interrupt priority = 0 in NVIC */
	// Relevant register: NVIC->IP[1], or use NVIC_SetPriority
	NVIC_SetPriority(EXTI0_1_IRQn, 0);

	/* Enable EXTI1 interrupts in NVIC */
	// Relevant register: NVIC->ISER[0], or use NVIC_EnableIRQ
	NVIC_EnableIRQ(EXTI0_1_IRQn);

}

/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void TIM2_IRQHandler()
{
	/* Check if update interrupt flag is indeed set */
	if ((TIM2->SR & TIM_SR_UIF) != 0)
	{
		trace_printf("\n*** Overflow! ***\n");

		/* Clear update interrupt flag */
		// Relevant register: TIM2->SR
		TIM2->SR &= ~(TIM_SR_UIF);

		/* Restart stopped timer */
		// Relevant register: TIM2->CR1
		TIM2->CR1 |= TIM_CR1_CEN;
	}
}


/* This handler is declared in system/src/cmsis/vectors_stm32f0xx.c */
void EXTI0_1_IRQHandler()
{
	// Your local variables...
	uint16_t timer_on = (TIM2->CR1 & TIM_CR1_CEN);
	float calc_period;
	/* Check if EXTI1 interrupt pending flag is indeed set */
	if ((EXTI->PR & EXTI_PR_PR1) != 0)
	{
		// 1. If this is the first edge:
		if(!timer_on)
		{
			//	- Clear count register (TIM2->CNT).
			TIM2->CNT = (uint32_t)0x0;

			//	- Start timer (TIM2->CR1).
			TIM2->CR1 |= TIM_CR1_CEN;

		} else {
			//    Else (this is the second edge):
			//	- Stop timer (TIM2->CR1).
			TIM2->CR1 &= ~(TIM_CR1_CEN);

			//	- Read out count register (TIM2->CNT).
			uint32_t count = TIM2->CNT;

			//	- Calculate signal period and frequency.
			calc_freq = ((float)SystemCoreClock) / count;
			calc_period = 1.0 / calc_freq;

		}

		// 2. Clear EXTI1 interrupt pending flag (EXTI->PR).
		EXTI->PR |= EXTI_PR_PR1;
	}
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------