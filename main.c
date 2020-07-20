#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <math.h>
#include <stdlib.h>

volatile uint8_t direction_X = 0, direction_Y = 0, direction_Z = 0; 

xQueueHandle xQueue;

void RCC_Init(void);
void USART2_Init(void);
void USART2_Send(char byte);
void Accel_Init(void);
void SPI_Init(void);
uint8_t Accel_Out(uint8_t reg);
uint8_t Accel_OutX_H(void);
uint8_t Accel_OutY_H(void);
uint8_t Accel_OutZ_H(void);
uint8_t Accel_OutX_L(void);
uint8_t Accel_OutY_L(void);
uint8_t Accel_OutZ_L(void);
void Itoa_Send(volatile uint8_t num);

uint8_t Angle_X(uint8_t OutX_H, uint8_t OutX_L, uint8_t OutY_H, uint8_t OutY_L, uint8_t OutZ_H, uint8_t OutZ_L);
uint8_t Angle_Y(uint8_t OutX_H, uint8_t OutX_L, uint8_t OutY_H, uint8_t OutY_L, uint8_t OutZ_H, uint8_t OutZ_L);


void vAccelerometerTask(void* param);
void vUSART2Task(void* param);


int main(){
	RCC_Init();
	USART2_Init();
	SPI_Init();
	Accel_Init();
	//Accel_Init();
	xQueue = xQueueCreate(4, sizeof(uint8_t));
	xTaskCreate(vAccelerometerTask, "Accel", 512, NULL, 1, NULL);
	xTaskCreate(vUSART2Task, "USART", 512, NULL, 1, NULL);
	
	vTaskStartScheduler();
	
	while(1){
	  //Errors here
	}
}

void RCC_Init(){ // initial frequency - 8 MHz	
	RCC->CR |= RCC_CR_HSEON; // turn on HSE
	while(!(RCC->CR & RCC_CR_HSERDY)) {}  // wait for HSE
	
	FLASH->ACR |= FLASH_ACR_LATENCY | FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN; //for flash
		
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSE;
	RCC->CR &= ~(RCC_CR_PLLON); // turn off HSE
	
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLM;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLN;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLQ;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;
	
	RCC->CFGR &= ~(RCC_CFGR_HPRE);
	RCC->CFGR &= ~(RCC_CFGR_PPRE1);
	RCC->CFGR &= ~(RCC_CFGR_PPRE2);
	
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV8;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLM_2; // div 4 (0100)
	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_3; // mul 168 (1010 1000)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_5;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLN_7;
	
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_0; // div 7 (0111)
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_1;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLQ_2;
	
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP; // div 2
	
  RCC->CR |= RCC_CR_PLLON; // turn on PLL
	while((RCC->CR & RCC_CR_PLLRDY) == 0);  // wait for PLL
	
	//RCC->CFGR &= ~(RCC_CFGR_SW);	
	RCC->CFGR |= RCC_CFGR_SW_PLL; // select PLL as a system clock
	while((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // wait for PLL is used
}


void USART2_Init(void){
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // for PA2(TX) and PA3(RX)
	
	//TX
	GPIOA->MODER &= ~GPIO_MODER_MODE2_1;
	GPIOA->MODER |= GPIO_MODER_MODE2_1; //alternate function mode (10)
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT2);  //push/pull
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD2);
	GPIOA->PUPDR |= GPIO_PUPDR_PUPD2_0; //01 - pull-up
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR2);
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2; //11 - 50 MHz
	GPIOA->AFR[0] |= (0x07 << (2*4));
	
	//RX
	GPIOA->MODER |= GPIO_MODER_MODE3_1;
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD3);
	GPIOA->AFR[0] |= (0x07 << (4 * 3));
	
	USART2->CR1 |= USART_CR1_UE;
	USART2->CR1 &= ~(USART_CR1_M); // 8 bits
	
	//USART2->BRR = 0x683; // 16 MHz
	USART2->BRR = 0x88C; // 42 MHz
	//USART2->BRR = 0x222E; // 84 MHz
	
  USART2->CR1 |= USART_CR1_RE;
	USART2->CR1 |= USART_CR1_TE;
}


void USART2_Send(char byte){
	while(!(USART2->SR & USART_SR_TC)){} // 
		USART2->DR = byte;
}

void SPI_Init(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	
	GPIOA->MODER |= GPIO_MODER_MODE5_1; // 10 - alternative function
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED5; // 11 - 50 MHz
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT5; // output push/pull (reset state)
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD5; // no pull up/down
	GPIOA->AFR[0] |= (0x05 << (5 * 4)); // alternative function number
	
  GPIOA->MODER |= GPIO_MODER_MODE6_1; // 10 - alternative function
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED6; // 11 - 50 MHz
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT6; 
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6; 
	GPIOA->AFR[0] |= (0x05 << (6 * 4));
	
	GPIOA->MODER |= GPIO_MODER_MODE7_1; // 10 - alternative function
	GPIOA->OSPEEDR |= GPIO_OSPEEDR_OSPEED7; // 11 - 50 MHz
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT7; 
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD7; 
	GPIOA->AFR[0] |= (0x05 << (7 * 4));
	
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
	
	GPIOE->MODER |= GPIO_MODER_MODE3_0; // 01 - output
	GPIOE->OSPEEDR |= GPIO_OSPEEDR_OSPEED3; // 11 - 50 MHz
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT3; 
	GPIOE->PUPDR &= ~GPIO_PUPDR_PUPD3; 
	
	GPIOE->ODR |= GPIO_ODR_OD3;
	
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	
	SPI1->CR1 &= ~SPI_CR1_SPE;
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE; // bidirectional mode
	SPI1->CR1 |= SPI_CR1_MSTR; // MASTER
	SPI1->CR1 &= ~SPI_CR1_DFF; // 8 bit
	SPI1->CR1 |= SPI_CR1_CPOL; // 1 is default
	SPI1->CR1 |= SPI_CR1_CPHA; // 1 - back front
	SPI1->CR1 |= SPI_CR1_SSI;
	SPI1->CR1 |= SPI_CR1_SSM; // software slave management enabled
	SPI1->CR1 |= SPI_CR1_BR_0; // 101 - Baud rate 64 MHz
	SPI1->CR1 |= SPI_CR1_BR_2;
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // first bit is HIGH
	SPI1->CRCPR = 0x07; // CRC Polunomal ???????????
//	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR1 |= SPI_CR1_SPE; // enable SPI
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
}



void Accel_Init(void){
	volatile uint16_t data;
	GPIOE->ODR &= ~GPIO_ODR_OD3; 
	SPI1->DR = 0x20;
	while(!(SPI1->SR & SPI_SR_RXNE));
	data = (uint8_t)SPI1->DR;
	SPI1->DR = 0x97;
  while(!(SPI1->SR & SPI_SR_RXNE));
	data = (uint8_t)SPI1->DR;
	GPIOE->ODR |= GPIO_ODR_OD3;
}

uint8_t Accel_Out(uint8_t reg){
	volatile uint8_t out;
 	GPIOE->ODR &= ~GPIO_ODR_OD3; 
	SPI1->DR = reg;
	while(!(SPI1->SR & SPI_SR_RXNE));
	out = (uint8_t)SPI1->DR;
	SPI1->DR = 0x00 ;
	while(!(SPI1->SR & SPI_SR_RXNE));
	out = (uint8_t)SPI1->DR;
	GPIOE->ODR |= GPIO_ODR_OD3;
	return out;
}

void vAccelerometerTask(void* param){
	volatile char ch = 0, sign = 0;
	while(1){
		
		
	  ch = Angle_X(Accel_Out(0xA9), Accel_Out(0xA8), Accel_Out(0xAB), Accel_Out(0xAA), Accel_Out(0xAD), Accel_Out(0xAC));
		
		if(direction_Y){
			sign = '-';
		}
		else{
			sign = '+';
		}
		
		xQueueSend(xQueue, &sign, 0);
		xQueueSend(xQueue, &ch, 0);
		
		ch = Angle_Y(Accel_Out(0xA9), Accel_Out(0xA8), Accel_Out(0xAB), Accel_Out(0xAA), Accel_Out(0xAD), Accel_Out(0xAC));
		
		if(direction_X){
			sign = '-';
		}
		else{
			sign = '+';
		}	
		
		xQueueSend(xQueue, &sign, 0);
		xQueueSend(xQueue, &ch, 0);
		
		vTaskDelay(1000);
	}
}

void vUSART2Task(void* param){
	volatile char ch = 0, sign = 0;
	volatile uint8_t i;
	while(1){	
		USART2_Send(' ');
		USART2_Send('X');
		USART2_Send(':');
		USART2_Send(' ');

		xQueueReceive(xQueue, &sign, 0);
		xQueueReceive(xQueue, &ch, 0);
		
		USART2_Send(sign);
		Itoa_Send(ch);
		
		USART2_Send(' ');
		USART2_Send('Y');
		USART2_Send(':');
		USART2_Send(' ');
		
		xQueueReceive(xQueue, &sign, 0);
		xQueueReceive(xQueue, &ch, 0);		
		
		USART2_Send(sign);
	  Itoa_Send(ch);

	  vTaskDelay(1000);
	}
}

 uint8_t Angle_X(uint8_t OutX_H, uint8_t OutX_L, uint8_t OutY_H, uint8_t OutY_L, uint8_t OutZ_H, uint8_t OutZ_L){
	volatile uint16_t OutX = 0, OutY = 0, OutZ = 0;
	volatile double angle;
	OutX |= OutX_H << 8;
	OutX |= OutX_L;
	OutY |= OutY_H << 8;
	OutY |= OutY_L;
	OutZ |= OutZ_H << 8;
	OutZ |= OutZ_L;

	if(OutX >= 0x8000){
		OutX = 0xFFFF - OutX;
		direction_X = 1;
	}
	else{
		direction_X = 0;
	}
	 
	if(OutY >= 0x8000){
		OutY = 0xFFFF - OutY;
		direction_Y = 1;
	}
	else{
		direction_Y = 0;
	}
	
	if(OutZ >= 0x8000){
		OutZ = 0xFFFF - OutZ;
		direction_Z = 1;
	}
	else{
		direction_Z = 0;
	}
	
	if(OutZ == 0){
		return 0;
	}
	angle = (double)atan((double)OutY/sqrt(OutX * OutX + OutZ * OutZ)); // rad
	angle = angle * 57.29578;
	
	if(direction_Z == 1){
		angle = 180 - angle;
	}
	return (uint8_t)angle;
}

uint8_t Angle_Y(uint8_t OutX_H, uint8_t OutX_L, uint8_t OutY_H, uint8_t OutY_L, uint8_t OutZ_H, uint8_t OutZ_L){
	volatile uint16_t OutX = 0, OutY = 0, OutZ = 0;
	volatile double angle;
	OutX |= OutX_H << 8;
	OutX |= OutX_L;
	OutY |= OutY_H << 8;
	OutY |= OutY_L;
	OutZ |= OutZ_H << 8;
	OutZ |= OutZ_L;
	
	if(OutX >= 0x8000){
		OutX = 0xFFFF - OutX;
		direction_X = 1;
	}
	else{
		direction_X = 0;
	}
	
  if(OutY >= 0x8000){
		OutY = 0xFFFF - OutY;
		direction_Y = 1;
	}
	else{
		direction_Y = 0;
	}
	
	if(OutZ >= 0x8000){
		OutZ = 0xFFFF - OutZ;
		direction_Z = 1;
	}
	else{
		direction_Z = 0;
	}
	
	
	if(OutZ == 0){
		return 0;
	}
	angle = (double)atan((double)OutX/ sqrt(OutY * OutY + OutZ * OutZ)); // rad
	angle = angle * 57.29578;
	
	if(direction_Z == 1){
		angle = 180 - angle;
	}
	return (uint8_t)angle;
}

void Itoa_Send(volatile uint8_t num) {
	volatile char res[] = { '0', '0', '0' };
	volatile uint8_t temp, i; 
	for (i = 0; i < 3; i++) {
		if (num == 0) {
			break;
		}
		temp = num;
		num /= 10;
		num *= 10;
		res[2 - i] = (temp - num) + '0';
		num /= 10;
	}
	for (i = 0; i < 3; i++) {
		USART2_Send(res[i]);
	}
}

