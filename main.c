#include "stm32f30x.h"
#include "system_stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_exti.h"
#include "stm32f30x_adc.h"
#include "stm32f30x_dma.h"
#include "stm32f30x_misc.h"

#define F_CPU 8000000UL
#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

#define receive_data_buffer_size 4;	
#define transmit_data_buffer_size 21;		

																						/* Data */ 
uint8_t transmitData[transmit_data_buffer_size]={0};
uint8_t receivedData[receive_data_buffer_size]={0};
uint16_t ADC1_Buffer[4]={0};
uint16_t ADC2_Buffer[4]={0};
		
float adcs[8]={0.0};

uint8_t PWM_DUTY1=0;
uint8_t PWM_DUTY2=0;
uint8_t PWM_DUTY3=0;
uint8_t PWM_DUTY4=0;
uint16_t CURRENT_M1=0xFFFF;
uint16_t CURRENT_M2=0xFFFF;
uint16_t CURRENT_M3=0xFFFF;
uint16_t CURRENT_M4=0xFFFF;
uint16_t ADC_TEST=0xFFFF;
uint16_t VBAT_ADC=0xFFFF;
uint8_t system_state=0;
uint8_t crc8=0xff;
uint8_t cycle_counter=0;

uint8_t transmit_data_counter=0; 
uint8_t received_data_counter=0;
uint8_t k=0;
 

/* functions */ 

void USART3_IRQHandler(void);
void PWM_config(void);
void USART_config(void);
void TX_buffer_init(void);
void delay(uint32_t time_delay);
void ADC12_DMA_START(void);
void set_pwm_duty(void);


int main(void){

			__enable_irq();
			ADC12_DMA_START();
			TX_buffer_init();
			USART_config();
			PWM_config();

	while(1){

		__NOP();

		for (k=0;k<4;k++){

			adcs[k]=(float)ADC1_Buffer[k]/4096*3.315;
			adcs[k+4]=(float)ADC2_Buffer[k]/4096*3.315;
		}

		// Duty cycles = X/255 *100%

		

		transmitData[2]=receivedData[0];
		transmitData[3]=receivedData[1];
		transmitData[4]=receivedData[2];
		transmitData[5]=receivedData[3];

	}
}

void USART3_IRQHandler(void){
	uint8_t j=0;
	if (USART3->ISR & USART_ISR_TXE)
        {
				USART3->TDR = transmitData[transmit_data_counter];
					
				crc8 ^= transmitData[transmit_data_counter];
				transmit_data_counter++;
					
				for (j = 0; j < 8; j++) {
						if ((crc8 & 0x80) != 0)
								crc8 = (uint8_t)((crc8<< 1) ^ 0x31);
						else
								crc8 <<= 1;
				}
								
				if (transmit_data_counter>(sizeof(transmitData)-1)){
					
						transmit_data_counter=0;
						
						transmitData[19]+=1;
						transmitData[20]=crc8;
						crc8=0xff;
				}
		}
	if (USART3->ISR & USART_ISR_RXNE) {
		
				
        uint8_t data = USART3->RDR;
        if (received_data_counter >= receive_data_buffer_size) {
             received_data_counter=0;
        }
        receivedData[received_data_counter] = data;
        received_data_counter++;
    }
}

uint8_t gencrc(uint8_t *data, uint8_t len){
    uint8_t crc = 0xff;
    uint8_t i, j;
    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if ((crc & 0x80) != 0)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc <<= 1;
        }
    }
    return crc;
}

void set_pwm_duty(uint8_t PWM_DUTY1, uint8_t PWM_DUTY2, uint8_t PWM_DUTY3, uint8_t PWM_DUTY4){
	
	if PWM_DUTY1
		TIM2->CCR1 = PWM_DUTY1;
		TIM2->CCR2 = PWM_DUTY2;
		TIM2->CCR3 = PWM_DUTY3;
		TIM2->CCR4 = PWM_DUTY4;	
	
	
}

	

void PWM_config(void){

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef        TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Prescaler =4;
	TIM_TimeBaseStructure.TIM_Period = 255;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 1100;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	TIM_CtrlPWMOutputs(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void USART_config(){
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	// PC10 -> TX UART.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//PC11  -> RX UART.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_7);

	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);


	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART3, &USART_InitStructure);
	USART_Cmd(USART3, ENABLE);
	NVIC_EnableIRQ(USART3_IRQn);

	USART3->CR1 |= USART_CR1_TXEIE;
	USART3->CR1 |= USART_CR1_RXNEIE;
}

void TX_buffer_init(void){

	transmitData[0]=0x0D;
	transmitData[1]=0x0A;
	transmitData[2]=PWM_DUTY1;
	transmitData[3]=PWM_DUTY2;
	transmitData[4]=PWM_DUTY3;
	transmitData[5]=PWM_DUTY4;

	transmitData[6] = 0x000000FF & (CURRENT_M1 >> 8);
	transmitData[7] = 0x000000FF & (CURRENT_M1);


	transmitData[8] = 0x000000FF & (CURRENT_M2 >> 8);
	transmitData[9] = 0x000000FF & (CURRENT_M2);


	transmitData[10] = 0x000000FF & (CURRENT_M3 >> 8);
	transmitData[11] = 0x000000FF & (CURRENT_M3);


	transmitData[12] = 0x000000FF & (CURRENT_M4 >> 8);
	transmitData[13] = 0x000000FF & (CURRENT_M4);


	transmitData[14] = 0x000000FF & (ADC_TEST >> 8);
	transmitData[15] = 0x000000FF & (ADC_TEST);


	transmitData[16] = 0x000000FF & (VBAT_ADC >> 8);
	transmitData[17] = 0x000000FF & (VBAT_ADC);

	transmitData[18]=0;
	transmitData[19]=cycle_counter;
	transmitData[20]=crc8;
}

/*
void NVIC_Config(void) {
NVIC_InitTypeDef NVIC_InitStructure;


    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void DMA1_Channel1_IRQHandler (void){
    DMA1->IFCR |= DMA_IFCR_CGIF1;
    //..............
    ADC1->CR |= ADC_CR_ADSTART; //start adc
}

void SysTick_Handler(void)
{
/* USER CODE BEGIN SysTick_IRQn 0

  timer1++;
}
*/

void delay(uint32_t time_delay){

    uint32_t i;
    for(i = 0; i < time_delay; i++);
}

void ADC12_DMA_START(void){

	GPIOA->MODER |= GPIO_MODER_MODER0 |GPIO_MODER_MODER1 |GPIO_MODER_MODER2 |GPIO_MODER_MODER3 |GPIO_MODER_MODER4 |GPIO_MODER_MODER5 |GPIO_MODER_MODER6 |GPIO_MODER_MODER7;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	RCC->AHBENR |= RCC_AHBENR_DMA2EN;

	//ADC1 DMA1

	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel1, ENABLE);

	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	delay(110);
	ADC1_2->CCR &= ~(ADC12_CCR_CKMODE_0 | ADC12_CCR_CKMODE_1);
	ADC1_2->CCR |= ADC12_CCR_CKMODE_0;
	ADC1->CR = 0;
	ADC1->CR &= ~ADC_CR_ADVREGEN;
	ADC1->CR |= ADC_CR_ADVREGEN; //ADC_CR_ADVREGEN_0;
	delay(110);
	ADC1->CR |= ADC_CR_ADCAL;
	while ((ADC1->CR & ADC_CR_ADCAL) != 0);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_NbrOfRegChannel = 4;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_OverrunMode = ENABLE;
	ADC_InitStructure.ADC_AutoInjMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_1,1, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2,2, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3,3, ADC_SampleTime_1Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4,4, ADC_SampleTime_1Cycles5);
  ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);

	ADC1->CR |= ADC_CR_ADEN;
	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC1, ENABLE);
	ADC_StartConversion(ADC1);

	/// ADC2 DMA2

	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC2_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_BufferSize = 4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC2->DR;
	DMA_Init(DMA2_Channel1, &DMA_InitStructure);
	DMA_Cmd(DMA2_Channel1, ENABLE);

	ADC2->CR = 0;
	ADC2->CR &= ~ADC_CR_ADVREGEN;
	ADC2->CR |= ADC_CR_ADVREGEN; //ADC_CR_ADVREGEN_0;

	delay(110);
	ADC2->CR |= ADC_CR_ADCAL;
	while ((ADC2->CR & ADC_CR_ADCAL) != 0);


	ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Enable;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_NbrOfRegChannel = 4;
	ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigConvEvent_0;
	ADC_InitStructure.ADC_OverrunMode = ENABLE;
	ADC_InitStructure.ADC_AutoInjMode = DISABLE;
	ADC_Init(ADC2, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC2, ADC_Channel_1,1, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_2,2, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_3,3, ADC_SampleTime_601Cycles5);
	ADC_RegularChannelConfig(ADC2, ADC_Channel_4,4, ADC_SampleTime_601Cycles5);
	ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC2, ENABLE);

	ADC2->CR |= ADC_CR_ADEN;
	ADC_DMAConfig(ADC2, ADC_DMAMode_Circular);
	ADC_DMACmd(ADC2, ENABLE);
	ADC_StartConversion(ADC2);
}

