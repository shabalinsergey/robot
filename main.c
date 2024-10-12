#include "stm32f30x.h"
#include "system_stm32f30x.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_tim.h"
#include "stm32f30x_usart.h"
#include "stm32f30x_exti.h"


#include "stm32f30x_misc.h"

#define F_CPU 8000000UL

// ѕока все идет точно также, как и в случае с передачей
/*
// «адаем структуру параметров порта
//void print(const char* str);
int main(void)
{

	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);


	//RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

		//GPIO_InitTypeDef GPIO_InitStructure;

	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	//GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	//GPIO_Init(GPIOB, &GPIO_InitStructure);
	//GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_1);

	 //TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	 //TIM_OCInitTypeDef        TIM_OCInitStructure;

	 // «аполн€ем структуру
	 //TIM_TimeBaseStructure.TIM_Prescaler = 10;
	 //TIM_TimeBaseStructure.TIM_Period = 511;
	 //TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	 //TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	 //TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	 //TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	 //TIM_OCInitStructure.TIM_Pulse = 360;
	 //TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	 //TIM_CtrlPWMOutputs(TIM2, ENABLE);							//enable the PWM output
	 //TIM_Cmd(TIM2, ENABLE);





	 //RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	// USART_InitTypeDef USART_InitStructure;
	 //USART_InitStructure.USART_BaudRate = 115200;

	 	 //USART_InitStructure.USART_WordLength = USART_WordLength_8b;

	 	// USART_InitStructure.USART_StopBits = USART_StopBits_1;

	 	 //USART_InitStructure.USART_Parity = USART_Parity_No ;

	 	 //USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	 	 //USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	 	//USART_Init(USART1, &USART_InitStructure);

	 	 //USART_Cmd(USART1, ENABLE);

	 //RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);







	  //GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;

	  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;


	  //GPIO_Init(GPIOC, &GPIO_InitStructure);



*/
void ADCInit(void);
void DMAInit(void);

// “елеметрируемые параметры
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

uint8_t transmitData[21]={0};













uint8_t full_word_transmitted=0;
uint8_t checksum=0;

uint8_t receivedData[16];
uint8_t bytesToReceive = 16;
// —четчик прин€тых байт
uint8_t receivedDataCounter = 0;
uint8_t i=0;
uint8_t j=0;

int main(void){
	__enable_irq();
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






	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_7);
	//заполн€ем пол€ структуры
	// PC10 -> TX UART.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); //инициализируем

	//PC11  -> RX UART.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//инициализируем


	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //–азрешаем тактирование


	  USART_InitStructure.USART_BaudRate = 115200;// скорость
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8 бит данных
	  USART_InitStructure.USART_StopBits = USART_StopBits_1; //один стоп бит
	  USART_InitStructure.USART_Parity = USART_Parity_No; //четность - нет
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // управлени потоком - нет
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       // разрешаем прием и передачу

	//USART_OverSampling8Cmd(ENABLE); //можно уменьшить частоту семплировани€
	//USART_OneBitMethodCmd(ENABLE); //можно уменьшить количество стробирований
	//USART_HalfDuplexCmd (ENABLE); // можно выбрать полудуплексный режим.

	  USART_Init(USART3, &USART_InitStructure); //инизиализируем
	  USART_Cmd(USART3, ENABLE);
	  NVIC_EnableIRQ(USART3_IRQn);
	 USART3->CR1 |= USART_CR1_TXEIE;
	 USART3->CR1 |= USART_CR1_RXNEIE;
	 // –азрешить прерывание по окончании передачи
	//USART3->CR1 |= USART_CR1_TCIE; // –азрешить прерывание по завершении передачи
	//USART3->ICR |= USART_ICR_TCCF; // —бросить флаг завершени€ передачи

	while(1){

		__NOP();
	}
}

void USART3_IRQHandler(void)
{
if (USART3->ISR & USART_ISR_TXE) // ≈сли регистр передачи пустой
        {

				USART3->TDR = transmitData[i]; // ѕередать символ 'A'
				crc8 ^= transmitData[i];
				i++;
				        for (j = 0; j < 8; j++) {
				            if ((crc8 & 0x80) != 0)
				                crc8 = (uint8_t)((crc8 << 1) ^ 0x31);
				            else
				                crc8 <<= 1;
				        }
				if (i>(sizeof(transmitData)-1)){
					i=0;
					transmitData[19]+=1;
					transmitData[20]=crc8;
					crc8=0xff;
				}
		}

}

void ADCInit(void){
	GPIO_InitTypeDef   GPIO_InitStructure;
		ADC_InitTypeDef    ADC_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;

		/* Enable the GPIOC Clock */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

		/* Configure the ADC clock */
		RCC_ADCCLKConfig(RCC_ADC12PLLCLK_Div2);

		/* ADC1 Periph clock enable */
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);

		/* Configure PC.1 (ADC Channel7) in analog mode */
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1| GPIO_Pin_2| GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5| GPIO_Pin_6| GPIO_Pin_7;;
		GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AN;
		GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
		GPIO_Init(ADC_PORT, &GPIO_InitStructure);

		ADC_StructInit(&ADC_InitStructure);

		/* Calibration procedure */
		ADC_VoltageRegulatorCmd(ADC1, ENABLE);

		/* Insert delay equal to 10 us */
		delay_us(10);

		ADC_SelectCalibrationMode(ADC1, ADC_CalibrationMode_Single);
		ADC_StartCalibration(ADC1);

		while(ADC_GetCalibrationStatus(ADC1) != RESET );
		calibration_value = ADC_GetCalibrationValue(ADC1);

		/* Configure the ADC1 in continuous mode */
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
		ADC_CommonInitStructure.ADC_Clock = ADC_Clock_SynClkModeDiv1;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
		ADC_CommonInitStructure.ADC_DMAMode = ADC_DMAMode_OneShot;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = 0;

		ADC_CommonInit(ADC1, &ADC_CommonInitStructure);

		ADC_InitStructure.ADC_ContinuousConvMode = ADC_ContinuousConvMode_Disable;
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ExternalTrigConvEvent = ADC_ExternalTrigInjecEventEdge_None; //ADC_ExternalTrigConvEvent_9;
		ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None; //ADC_ExternalTrigEventEdge_RisingEdge;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
		ADC_InitStructure.ADC_OverrunMode = ADC_OverrunMode_Disable;
		ADC_InitStructure.ADC_AutoInjMode = ADC_AutoInjec_Disable;
		ADC_InitStructure.ADC_NbrOfRegChannel = 4;
		ADC_Init(ADC1, &ADC_InitStructure);

		/* ADC1 regular channel7 configuration */
		ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_181Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 2, ADC_SampleTime_181Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_181Cycles5);
		ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 4, ADC_SampleTime_181Cycles5);

		/* Enable ADC1 */
		ADC_Cmd(ADC1, ENABLE);

		/* wait for ADRDY */
		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY));

		/* ADC1 DMA Enable */
		ADC_DMACmd(ADC1, ENABLE);
		ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);

		/* Start ADC1 Software Conversion */
		ADC_StartConversion(ADC1);
}

void DMA_Config(void)
{
	DMA_InitTypeDef  DMA_InitStructure;
  	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable DMA1 clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)ADC1_DR_ADDRESS;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&io.adc_sample;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 Channel1 Transfer Complete interrupt */
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

	/* Enable DMA1 channel1 IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable DMA1 Channel1 transfer */
	DMA_Cmd(DMA1_Channel1, ENABLE);
}



uint8_t gencrc(uint8_t *data, uint8_t len)
{
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


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %drn", file, line) */

  /* Infinite loop */

}



#endif
