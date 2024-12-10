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


void PWM_config(void);
void USART_config(void);
void TX_buffer_init(void);

// Р В РЎС›Р В Р’ВµР В Р’В»Р В Р’ВµР В РЎпїЅР В Р’ВµР РЋРІР‚С™Р РЋР вЂљР В РЎвЂ�Р РЋР вЂљР РЋРЎвЂњР В Р’ВµР В РЎпїЅР РЋРІР‚в„–Р В Р’Вµ Р В РЎвЂ”Р В Р’В°Р РЋР вЂљР В Р’В°Р В РЎпїЅР В Р’ВµР РЋРІР‚С™Р РЋР вЂљР РЋРІР‚в„–
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
uint8_t buffer_index=0;
uint8_t rx_buffer[4];
uint8_t BUFFER_SIZE=4;
uint8_t receivedData[16];
uint8_t bytesToReceive =4;
uint8_t index=0;
// Р В Р Р‹Р РЋРІР‚РЋР В Р’ВµР РЋРІР‚С™Р РЋРІР‚РЋР В РЎвЂ�Р В РЎвЂќ Р В РЎвЂ”Р РЋР вЂљР В РЎвЂ�Р В Р вЂ¦Р РЋР РЏР РЋРІР‚С™Р РЋРІР‚в„–Р РЋРІР‚В¦ Р В Р’В±Р В Р’В°Р В РІвЂћвЂ“Р РЋРІР‚С™
uint8_t receivedDataCounter = 0;
uint8_t i=0;
uint8_t j=0;
uint16_t ADC_Buffer[4]={0};
uint8_t ADC_BUFFER_SIZE=4;


int main(void){

			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
			//DMA_Config();
			 //dma1Init();
			__enable_irq(); // Р В Р’В Р В Р’В°Р В Р’В·Р РЋР вЂљР В Р’ВµР РЋРІвЂљВ¬Р В Р’В°Р В Р’ВµР В РЎпїЅ Р В РЎвЂ”Р РЋР вЂљР В Р’ВµР РЋР вЂљР РЋРІР‚в„–Р В Р вЂ Р В Р’В°Р В Р вЂ¦Р В РЎвЂ�Р РЋР РЏ
			 //adc12Init();
			test_fcn();
			//ADC_Start();
			TX_buffer_init(); // Р В РїС—Р…Р В Р вЂ¦Р В РЎвЂ�Р РЋРІР‚В Р В РЎвЂ�Р В Р’В°Р В Р’В»Р В РЎвЂ�Р В Р’В·Р В Р’В°Р РЋРІР‚В Р В РЎвЂ�Р РЋР РЏ Р В Р’В±Р РЋРЎвЂњР РЋРІР‚С›Р РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В Р’В° Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚С™Р РЋРІР‚РЋР В РЎвЂ�Р В РЎвЂќР В Р’В° USART
			USART_config(); // Р В РїС—Р…Р В Р вЂ¦Р В РЎвЂ�Р РЋРІР‚В Р В РЎвЂ�Р В Р’В°Р В Р’В»Р В РЎвЂ�Р В Р’В·Р В Р’В°Р РЋРІР‚В Р В РЎвЂ�Р РЋР РЏ USART: RX/TX. baudrate 115200, data bits=8, parity=none,  stop bits=1
			PWM_config(); // Р В РїС—Р…Р В Р вЂ¦Р В РЎвЂ�Р РЋРІР‚В Р В РЎвЂ�Р В Р’В°Р В Р’В»Р В РЎвЂ�Р В Р’В·Р В Р’В°Р РЋРІР‚В Р В РЎвЂ�Р РЋР РЏ PWM1-PWM4, F=6.25kHz, duty=50%

	while(1){

		__NOP();

		//transmitData[2]=rx_buffer[0];
		//transmitData[3]=rx_buffer[1];
		//transmitData[4]=rx_buffer[2];
		//transmitData[5]=rx_buffer[3];

		//TIM2->CCR1 = 127;
		//TIM2->CCR2 = 127;
		//TIM2->CCR3 = 127;
		//TIM2->CCR4 = 127;

		transmitData[2]=rx_buffer[0];
		transmitData[3]=rx_buffer[1];
		transmitData[4]=rx_buffer[2];
		transmitData[5]=rx_buffer[3];
}
}
void USART3_IRQHandler(void)
{
if (USART3->ISR & USART_ISR_TXE) // Р В РІР‚СћР РЋР С“Р В Р’В»Р В РЎвЂ� Р РЋР вЂљР В Р’ВµР В РЎвЂ“Р В РЎвЂ�Р РЋР С“Р РЋРІР‚С™Р РЋР вЂљ Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚РЋР В РЎвЂ� Р В РЎвЂ”Р РЋРЎвЂњР РЋР С“Р РЋРІР‚С™Р В РЎвЂўР В РІвЂћвЂ“
        {

				USART3->TDR = transmitData[i]; // Р В РЎСџР В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚С™Р РЋР Р‰ Р РЋР С“Р В РЎвЂ�Р В РЎпїЅР В Р вЂ Р В РЎвЂўР В Р’В» 'A'
				crc8 ^= transmitData[i];
				i++;
				        for (j = 0; j < 8; j++) {
				            if ((crc8 & 0x80) != 0)
				                crc8 = (uint8_t)((crc8<< 1) ^ 0x31);
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
	if (USART3->ISR & USART_ISR_RXNE) {
        uint8_t data = USART3->RDR;
        if (buffer_index >= BUFFER_SIZE) {
             buffer_index=0;
        }
        rx_buffer[buffer_index] = data;
        buffer_index++;
    }
}


void ADC_Start(){

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // Р Р°Р·СЂРµС€РёС‚СЊ С‚Р°РєС‚РёСЂРѕРІР°РЅРёРµ РїРѕСЂС‚Р° PORTA
	GPIOA->MODER |= GPIO_MODER_MODER0; //РђРЅР°Р»РѕРіРѕРІС‹Р№ РІС…РѕРґ PA1/*

	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	delay(110);
	RCC->AHBENR |= RCC_AHBENR_ADC12EN; //РїРѕРґР°РµРј С‚Р°РєС‚РёСЂРѕРІР°РЅРёРµ РђР¦Рџ
	ADC1_2->CCR &= ~(ADC12_CCR_CKMODE_0 | ADC12_CCR_CKMODE_1);
	ADC1_2->CCR |= ADC12_CCR_CKMODE_0;
	ADC1->CR = 0; // РќР° РІСЃСЏРєРёР№ СЃР»СѓС‡Р°Р№
	ADC1->CR &= ~ADC_CR_ADVREGEN; //Р­С‚Рѕ РІСЂРѕРґРµ РєР°Рє РІС‹РєР»СЋС‡РµРЅРёРµ СЂРµРіСѓР»СЏС‚РѕСЂР° РЅР°РїСЂСЏР¶РµРЅРёСЏ
	ADC1->CR |= ADC_CR_ADVREGEN_0; // | ADC_CR_ADVREGEN_1;
	delay(110);
	ADC1->CR |= ADC_CR_ADCAL; //Р—Р°РїСѓСЃРє РєР°Р»РёР±СЂРѕРІРєРё
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) {}; //РћР¶РёРґР°РЅРёРµ РµРµ РєРѕРЅС†Р°
	ADC1->CR |= ADC_CR_ADEN; //РІРєР»СЋС‡РёС‚СЊ РђР¦Рџ
	ADC1->SQR1 |= 1<<3; //ADC_SQR1_SQ1_0
	//ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);

	ADC_StartConversion(ADC1);



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

void PWM_config(void){
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

				 // Р В РІР‚вЂќР В Р’В°Р В РЎвЂ”Р В РЎвЂўР В Р’В»Р В Р вЂ¦Р РЋР РЏР В Р’ВµР В РЎпїЅ Р РЋР С“Р РЋРІР‚С™Р РЋР вЂљР РЋРЎвЂњР В РЎвЂќР РЋРІР‚С™Р РЋРЎвЂњР РЋР вЂљР РЋРЎвЂњ
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

				 TIM_CtrlPWMOutputs(TIM2, ENABLE);							//enable the PWM output
				 TIM_Cmd(TIM2, ENABLE);
}

void USART_config(){

	GPIO_InitTypeDef GPIO_InitStructure;
	// PC10 -> TX UART.
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure); //Р В РЎвЂ�Р В Р вЂ¦Р В РЎвЂ�Р РЋРІР‚В Р В РЎвЂ�Р В Р’В°Р В Р’В»Р В РЎвЂ�Р В Р’В·Р В РЎвЂ�Р РЋР вЂљР РЋРЎвЂњР В Р’ВµР В РЎпїЅ

	//PC11  -> RX UART.
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_Init(GPIOC, &GPIO_InitStructure);//Р В РЎвЂ�Р В Р вЂ¦Р В РЎвЂ�Р РЋРІР‚В Р В РЎвЂ�Р В Р’В°Р В Р’В»Р В РЎвЂ�Р В Р’В·Р В РЎвЂ�Р РЋР вЂљР РЋРЎвЂњР В Р’ВµР В РЎпїЅ

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_7);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_7);

	USART_InitTypeDef USART_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //Р В Р’В Р В Р’В°Р В Р’В·Р РЋР вЂљР В Р’ВµР РЋРІвЂљВ¬Р В Р’В°Р В Р’ВµР В РЎпїЅ Р РЋРІР‚С™Р В Р’В°Р В РЎвЂќР РЋРІР‚С™Р В РЎвЂ�Р РЋР вЂљР В РЎвЂўР В Р вЂ Р В Р’В°Р В Р вЂ¦Р В РЎвЂ�Р В Р’Вµ


	  USART_InitStructure.USART_BaudRate = 115200;// Р РЋР С“Р В РЎвЂќР В РЎвЂўР РЋР вЂљР В РЎвЂўР РЋР С“Р РЋРІР‚С™Р РЋР Р‰
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b; //8 Р В Р’В±Р В РЎвЂ�Р РЋРІР‚С™ Р В РўвЂ�Р В Р’В°Р В Р вЂ¦Р В Р вЂ¦Р РЋРІР‚в„–Р РЋРІР‚В¦
	  USART_InitStructure.USART_StopBits = USART_StopBits_1; //Р В РЎвЂўР В РўвЂ�Р В РЎвЂ�Р В Р вЂ¦ Р РЋР С“Р РЋРІР‚С™Р В РЎвЂўР В РЎвЂ” Р В Р’В±Р В РЎвЂ�Р РЋРІР‚С™
	  USART_InitStructure.USART_Parity = USART_Parity_No; //Р РЋРІР‚РЋР В Р’ВµР РЋРІР‚С™Р В Р вЂ¦Р В РЎвЂўР РЋР С“Р РЋРІР‚С™Р РЋР Р‰ - Р В Р вЂ¦Р В Р’ВµР РЋРІР‚С™
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; // Р РЋРЎвЂњР В РЎвЂ”Р РЋР вЂљР В Р’В°Р В Р вЂ Р В Р’В»Р В Р’ВµР В Р вЂ¦Р В РЎвЂ� Р В РЎвЂ”Р В РЎвЂўР РЋРІР‚С™Р В РЎвЂўР В РЎвЂќР В РЎвЂўР В РЎпїЅ - Р В Р вЂ¦Р В Р’ВµР РЋРІР‚С™
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;       // Р РЋР вЂљР В Р’В°Р В Р’В·Р РЋР вЂљР В Р’ВµР РЋРІвЂљВ¬Р В Р’В°Р В Р’ВµР В РЎпїЅ Р В РЎвЂ”Р РЋР вЂљР В РЎвЂ�Р В Р’ВµР В РЎпїЅ Р В РЎвЂ� Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚РЋР РЋРЎвЂњ

	  USART_Init(USART3, &USART_InitStructure); //Р В РЎвЂ�Р В Р вЂ¦Р В РЎвЂ�Р В Р’В·Р В РЎвЂ�Р В Р’В°Р В Р’В»Р В РЎвЂ�Р В Р’В·Р В РЎвЂ�Р РЋР вЂљР РЋРЎвЂњР В Р’ВµР В РЎпїЅ
	  USART_Cmd(USART3, ENABLE);
	  NVIC_EnableIRQ(USART3_IRQn);

	 USART3->CR1 |= USART_CR1_TXEIE;
	 USART3->CR1 |= USART_CR1_RXNEIE;
	 // Р В Р’В Р В Р’В°Р В Р’В·Р РЋР вЂљР В Р’ВµР РЋРІвЂљВ¬Р В РЎвЂ�Р РЋРІР‚С™Р РЋР Р‰ Р В РЎвЂ”Р РЋР вЂљР В Р’ВµР РЋР вЂљР РЋРІР‚в„–Р В Р вЂ Р В Р’В°Р В Р вЂ¦Р В РЎвЂ�Р В Р’Вµ Р В РЎвЂ”Р В РЎвЂў Р В РЎвЂўР В РЎвЂќР В РЎвЂўР В Р вЂ¦Р РЋРІР‚РЋР В Р’В°Р В Р вЂ¦Р В РЎвЂ�Р В РЎвЂ� Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚РЋР В РЎвЂ�
	//USART3->CR1 |= USART_CR1_TCIE; // Р В Р’В Р В Р’В°Р В Р’В·Р РЋР вЂљР В Р’ВµР РЋРІвЂљВ¬Р В РЎвЂ�Р РЋРІР‚С™Р РЋР Р‰ Р В РЎвЂ”Р РЋР вЂљР В Р’ВµР РЋР вЂљР РЋРІР‚в„–Р В Р вЂ Р В Р’В°Р В Р вЂ¦Р В РЎвЂ�Р В Р’Вµ Р В РЎвЂ”Р В РЎвЂў Р В Р’В·Р В Р’В°Р В Р вЂ Р В Р’ВµР РЋР вЂљР РЋРІвЂљВ¬Р В Р’ВµР В Р вЂ¦Р В РЎвЂ�Р В РЎвЂ� Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚РЋР В РЎвЂ�
	//USART3->ICR |= USART_ICR_TCCF; // Р В Р Р‹Р В Р’В±Р РЋР вЂљР В РЎвЂўР РЋР С“Р В РЎвЂ�Р РЋРІР‚С™Р РЋР Р‰ Р РЋРІР‚С›Р В Р’В»Р В Р’В°Р В РЎвЂ“ Р В Р’В·Р В Р’В°Р В Р вЂ Р В Р’ВµР РЋР вЂљР РЋРІвЂљВ¬Р В Р’ВµР В Р вЂ¦Р В РЎвЂ�Р РЋР РЏ Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚РЋР В РЎвЂ�
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


void GPIO_Config(void) {
    // Р В РІР‚в„ўР В РЎвЂќР В Р’В»Р РЋР вЂ№Р РЋРІР‚РЋР В Р’В°Р В Р’ВµР В РЎпїЅ Р РЋРІР‚С™Р В Р’В°Р В РЎвЂќР РЋРІР‚С™Р В РЎвЂ�Р РЋР вЂљР В РЎвЂўР В Р вЂ Р В Р’В°Р В Р вЂ¦Р В РЎвЂ�Р В Р’Вµ GPIOA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

    // Р В РЎСљР В Р’В°Р РЋР С“Р РЋРІР‚С™Р РЋР вЂљР В Р’В°Р В РЎвЂ�Р В Р вЂ Р В Р’В°Р В Р’ВµР В РЎпїЅ PA0 - PA3 Р В Р вЂ  Р В Р’В°Р В Р вЂ¦Р В Р’В°Р В Р’В»Р В РЎвЂўР В РЎвЂ“Р В РЎвЂўР В Р вЂ Р В РЎвЂўР В РЎпїЅ Р РЋР вЂљР В Р’ВµР В Р’В¶Р В РЎвЂ�Р В РЎпїЅР В Р’Вµ
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*
void DMA_Config(void) {

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_InitTypeDef DMA_InitStructure;
    DMA_DeInit(DMA1_Channel1);

    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);  // Р В РЎвЂ™Р В РўвЂ�Р РЋР вЂљР В Р’ВµР РЋР С“ Р РЋР вЂљР В Р’ВµР В РЎвЂ“Р В РЎвЂ�Р РЋР С“Р РЋРІР‚С™Р РЋР вЂљР В Р’В° Р В РўвЂ�Р В Р’В°Р В Р вЂ¦Р В Р вЂ¦Р РЋРІР‚в„–Р РЋРІР‚В¦ Р В РЎвЂ™Р В Р’В¦Р В РЎСџ
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Buffer;       // Р В РЎвЂ™Р В РўвЂ�Р РЋР вЂљР В Р’ВµР РЋР С“ Р В Р’В±Р РЋРЎвЂњР РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В Р’В°
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                 // Р В Р’В§Р РЋРІР‚С™Р В Р’ВµР В Р вЂ¦Р В РЎвЂ�Р В Р’Вµ Р В РЎвЂ�Р В Р’В· Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В РЎвЂ�Р РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В РЎвЂ�Р В РЎвЂ�
    DMA_InitStructure.DMA_BufferSize = ADC_BUFFER_SIZE;                // Р В Р’В Р В Р’В°Р В Р’В·Р В РЎпїЅР В Р’ВµР РЋР вЂљ Р В Р’В±Р РЋРЎвЂњР РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В Р’В° (4 Р В РЎвЂќР В Р’В°Р В Р вЂ¦Р В Р’В°Р В Р’В»Р В Р’В°)
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;   // Р В РЎвЂ™Р В РўвЂ�Р РЋР вЂљР В Р’ВµР РЋР С“ Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В РЎвЂ�Р РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В РЎвЂ�Р В РЎвЂ� Р В Р вЂ¦Р В Р’Вµ Р В РЎвЂ�Р В Р вЂ¦Р В РЎвЂќР РЋР вЂљР В Р’ВµР В РЎпїЅР В Р’ВµР В Р вЂ¦Р РЋРІР‚С™Р В РЎвЂ�Р РЋР вЂљР РЋРЎвЂњР В Р’ВµР РЋРІР‚С™Р РЋР С“Р РЋР РЏ
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;            // Р В РЎвЂ™Р В РўвЂ�Р РЋР вЂљР В Р’ВµР РЋР С“ Р В РЎвЂ”Р В Р’В°Р В РЎпїЅР РЋР РЏР РЋРІР‚С™Р В РЎвЂ� Р В РЎвЂ�Р В Р вЂ¦Р В РЎвЂќР РЋР вЂљР В Р’ВµР В РЎпїЅР В Р’ВµР В Р вЂ¦Р РЋРІР‚С™Р В РЎвЂ�Р РЋР вЂљР РЋРЎвЂњР В Р’ВµР РЋРІР‚С™Р РЋР С“Р РЋР РЏ
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // Р В Р’В Р В Р’В°Р В Р’В·Р В РЎпїЅР В Р’ВµР РЋР вЂљ Р В РўвЂ�Р В Р’В°Р В Р вЂ¦Р В Р вЂ¦Р РЋРІР‚в„–Р РЋРІР‚В¦ Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В РЎвЂ�Р РЋРІР‚С›Р В Р’ВµР РЋР вЂљР В РЎвЂ�Р В РЎвЂ� (16 Р В Р’В±Р В РЎвЂ�Р РЋРІР‚С™)
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;          // Р В Р’В Р В Р’В°Р В Р’В·Р В РЎпїЅР В Р’ВµР РЋР вЂљ Р В РўвЂ�Р В Р’В°Р В Р вЂ¦Р В Р вЂ¦Р РЋРІР‚в„–Р РЋРІР‚В¦ Р В РЎвЂ”Р В Р’В°Р В РЎпїЅР РЋР РЏР РЋРІР‚С™Р В РЎвЂ� (16 Р В Р’В±Р В РЎвЂ�Р РЋРІР‚С™)
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                    // Р В Р’В¦Р В РЎвЂ�Р В РЎвЂќР В Р’В»Р В РЎвЂ�Р РЋРІР‚РЋР В Р’ВµР РЋР С“Р В РЎвЂќР В РЎвЂ�Р В РІвЂћвЂ“ Р РЋР вЂљР В Р’ВµР В Р’В¶Р В РЎвЂ�Р В РЎпїЅ
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                // Р В РІР‚в„ўР РЋРІР‚в„–Р РЋР С“Р В РЎвЂўР В РЎвЂќР В РЎвЂ�Р В РІвЂћвЂ“ Р В РЎвЂ”Р РЋР вЂљР В РЎвЂ�Р В РЎвЂўР РЋР вЂљР В РЎвЂ�Р РЋРІР‚С™Р В Р’ВµР РЋРІР‚С™
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                       // Р В РІР‚пїЅР В Р’ВµР В Р’В· Р В РЎвЂ”Р В Р’ВµР РЋР вЂљР В Р’ВµР В РўвЂ�Р В Р’В°Р РЋРІР‚РЋР В РЎвЂ� "Р В РЎвЂ”Р В Р’В°Р В РЎпїЅР РЋР РЏР РЋРІР‚С™Р РЋР Р‰-Р В РЎвЂ”Р В Р’В°Р В РЎпїЅР РЋР РЏР РЋРІР‚С™Р РЋР Р‰"

    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
    //DMA_ITConfig(DMA1_Channel1_IRQn,ENABLE);
    // Р В РІР‚в„ўР В РЎвЂќР В Р’В»Р РЋР вЂ№Р РЋРІР‚РЋР В Р’В°Р В Р’ВµР В РЎпїЅ DMA Р В РўвЂ�Р В Р’В»Р РЋР РЏ Р В РЎвЂ™Р В Р’В¦Р В РЎСџ

    	DMA1_Channel1->CCR |= DMA_CCR_TCIE;            //
        DMA1_Channel1->CCR |= DMA_CCR_CIRC;            //

        DMA1->IFCR |= 0x0F;                            //Reset flags

        NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
        NVIC_EnableIRQ (DMA1_Channel1_IRQn);
        DMA1_Channel1->CCR |= DMA_CCR_EN;
}

void ADC_Config(void) {
    RCC_APB2PeriphClockCmd(RCC_AHBPeriph_ADC12, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOA, &GPIO_InitStructure);
    ADC_InitTypeDef ADC_InitStructure;
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigEventEdge = ADC_ExternalTrigEventEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfRegChannel=4;
        ADC_Init(ADC1, &ADC_InitStructure);


    	ADC1_2->CCR &= ~(ADC12_CCR_CKMODE_0 | ADC12_CCR_CKMODE_1);
    	ADC1_2->CCR |= ADC12_CCR_CKMODE_0;
    	ADC1->CR = 0; // РќР° РІСЃСЏРєРёР№ СЃР»СѓС‡Р°Р№
    	ADC1->CR &= ~ADC_CR_ADVREGEN;
    	ADC1->CR |= ADC_CR_ADVREGEN_0;
    	delay(110);
    	ADC1->CR |= ADC_CR_ADCAL;
    	while ((ADC1->CR & ADC_CR_ADCAL) != 0) {};


    ADC_RegularChannelConfig(ADC1, ADC_Channel_1,1, ADC_SampleTime_1Cycles5); // PA0 (ADC Channel 1)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_2,2, ADC_SampleTime_1Cycles5); // PA1 (ADC Channel 2)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3,3, ADC_SampleTime_1Cycles5); // PA2 (ADC Channel 3)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4,4, ADC_SampleTime_1Cycles5); // PA3 (ADC Channel 4)

    ADC_DMACmd(ADC1, ENABLE);


    ADC1->CR |= ADC_CR_ADEN; //РІРєР»СЋС‡РёС‚СЊ РђР¦Рџ
    ADC_DMAConfig(ADC1,ENABLE);


    ADC_StartConversion(ADC1);

}

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

void delay(uint32_t time_delay)
{
    uint32_t i;
    for(i = 0; i < time_delay; i++);
}



void adc12Init (void){

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER |= GPIO_MODER_MODER1;    //Analog mode
    GPIOA->MODER |= GPIO_MODER_MODER6;    //Analog mode


    RCC->AHBENR |= RCC_AHBENR_ADC12EN;   //ADC Clock Enable
    RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_NO; //ADC12 <- AHB clock
    ADC1_2->CCR |= (0x1<<16);            //00:(Asynchronous clock mode - PLL), 01: HCLK/1, 10: HCLK/2, 11: HCLK/4
    ADC1_2->CCR |= (4<<8);               //Delay
    ADC1_2->CCR |= (7<<0);               //Interleaved mode only


    //ADC voltage regulator reset
    ADC1->CR &=~ADC_CR_ADVREGEN;
    ADC2->CR &=~ADC_CR_ADVREGEN; delay(100);
    //ADC voltage regulator enable
    ADC1->CR |= ADC_CR_ADVREGEN_0;
    ADC2->CR |= ADC_CR_ADVREGEN_0;

    //Calibration adc1_2
    ADC1->CR |= ADC_CR_ADCAL;
    while(ADC1->CR & ADC_CR_ADCAL);
    ADC2->CR |= ADC_CR_ADCAL;
    while(ADC2->CR & ADC_CR_ADCAL);

    //adc1_2 enable
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRD));
    ADC2->CR |= ADC_CR_ADEN;
    while (!(ADC2->ISR & ADC_ISR_ADRD));


    ADC1->CFGR |=  (2<<3);  //DataResolution: 0 12-bit, 1 10-bit, 2 8-bit, 3 6-bit
    ADC2->CFGR |=  (2<<3);  //DataResolution: 0 12-bit, 1 10-bit, 2 8-bit, 3 6-bit


    ADC1->SQR1 |= ADC_SQR1_SQ1_0;                  //ADC1_IN1
    ADC2->SQR1 |= ADC_SQR1_SQ1_1 | ADC_SQR1_SQ1_0; //ADC2_IN3

    //Set sampling time for regular group 1
    ADC1->SMPR1 |= (0<<3); //1.5 ADC clock cycles
    ADC2->SMPR1 |= (0<<9); //1.5 ADC clock cycles

    //ADC1->CFGR |= ADC_CFGR_CONT;  //0: Single conversion mode, 1: Continuous conversion mode
    //ADC2->CFGR |= ADC_CFGR_CONT;  //0: Single conversion mode, 1: Continuous conversion mode
    ADC1->CFGR |= ADC_CFGR_OVRMOD;
    ADC2->CFGR |= ADC_CFGR_OVRMOD;

    //DMA
    ADC1_2->CCR |= (2<<14);   //MDMA  2: 12 and 10-bit resolution, 3: 8 and 6-bit resolution

    //External trigger
  //ADC1->CFGR |= (0x2<<10); //EXTEN
  //  ADC1->CFGR |= (0x4<<6);  //EXTSEL - TIM3


    ADC1->CR |= ADC_CR_ADSTART;
}

void test_fcn(void){


	GPIOA->MODER |= GPIO_MODER_MODER0 |GPIO_MODER_MODER1 |GPIO_MODER_MODER2 |GPIO_MODER_MODER3;
	RCC->AHBENR |= RCC_AHBENR_ADC12EN;
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	    DMA_InitTypeDef DMA_InitStructure;
		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_Buffer;
	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//DMA_PeripheralDataSize_Word;//
	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	    DMA_InitStructure.DMA_BufferSize = 4; // in data unit
	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	    DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	    DMA_Cmd(DMA1_Channel1, ENABLE);

	    	RCC->CFGR2 |= RCC_CFGR2_ADCPRE12_DIV1;
	    		    	delay(110);
	    	ADC1_2->CCR &= ~(ADC12_CCR_CKMODE_0 | ADC12_CCR_CKMODE_1);
	    		    	ADC1_2->CCR |= ADC12_CCR_CKMODE_0;
	    		    	ADC1->CR = 0; // РќР° РІСЃСЏРєРёР№ СЃР»СѓС‡Р°Р№
	    		    	ADC1->CR &= ~ADC_CR_ADVREGEN; //Р­С‚Рѕ РІСЂРѕРґРµ РєР°Рє РІС‹РєР»СЋС‡РµРЅРёРµ СЂРµРіСѓР»СЏС‚РѕСЂР° РЅР°РїСЂСЏР¶РµРЅРёСЏ
	    		    	ADC1->CR |= ADC_CR_ADVREGEN_0; // | ADC_CR_ADVREGEN_1;
	    		    	delay(110);
	    		    	ADC1->CR |= ADC_CR_ADCAL; //Р—Р°РїСѓСЃРє РєР°Р»РёР±СЂРѕРІРєРё
	    		    	while ((ADC1->CR & ADC_CR_ADCAL) != 0);


	    //while(ADC1->CR & ADC_CR_ADEN);
	    //ADC_DeInit(ADC1);
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

	    // Set up ADC channels

	        // make them regular channels with rank i +1, sample time is 19 ADC cycles

	        ADC_RegularChannelConfig(ADC1, ADC_Channel_1,1, ADC_SampleTime_1Cycles5);
	        ADC_RegularChannelConfig(ADC1, ADC_Channel_2,2, ADC_SampleTime_1Cycles5);
	        ADC_RegularChannelConfig(ADC1, ADC_Channel_3,3, ADC_SampleTime_1Cycles5);
	        ADC_RegularChannelConfig(ADC1, ADC_Channel_4,4, ADC_SampleTime_1Cycles5);

	    ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	    ADC_DMACmd(ADC1, ENABLE);
	    // Enable everything



	    	ADC1->CR |= ADC_CR_ADEN;
	    	ADC_DMAConfig(ADC1, ADC_DMAMode_Circular);
	    		        ADC_DMACmd(ADC1, ENABLE);
	   // ADC_Cmd(ADC1, ENABLE);
	    		        //ADC_Cmd(ADC1, ENABLE);
	    //while (!(ADC_GetFlagStatus(ADC1, ADC_FLAG_RDY)));
	    ADC_StartConversion(ADC1);


}
