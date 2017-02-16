/* Imported files */
#include <stdio.h>
#include <stdbool.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/* STM32F103 SPI2 output pin definition */
#define SPI_GPIO_Port	GPIOB
#define SPI_CSN				GPIO_Pin_12
#define SPI_SCK 			GPIO_Pin_13
#define SPI_MISO 			GPIO_Pin_14
#define SPI_MOSI 			GPIO_Pin_15
#define SPI_CE 				GPIO_Pin_10

/* Define static coefficients */
static uint16_t const deadZone = 400;
static uint16_t const sensitivity = 500;

/* Microcontroller setup, function declaration */
static void GPIO_initialize(void);
static void TIM_initialize(void);
static void NVIC_Configuration(void);
static void RCC_initialize(void);
static void ADC_initialize(void);

/* User function declaration*/
static uint16_t analog_reading(uint8_t);
uint16_t ADC_balance(uint16_t);


//Type Define
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_OCInitTypeDef outputChannelInit = {0,};	
ADC_InitTypeDef  ADC_InitStructure;

//Variables
uint16_t CCR_reset = 45;
uint16_t PrescalerValue = 0;

uint16_t LeftJoystick_Raw = 0;
uint16_t RightJoystick_Raw = 0;
uint16_t LeftJoystick_Position = 0;
uint16_t RightJoystick_Position = 0;


int main(void) {
	
	RCC_initialize();
	GPIO_initialize();
	NVIC_Configuration();	
	TIM_initialize();
	ADC_initialize();


  // Start Timer 2 interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	// LOOOOOOOOP
	while(1) {
		
		// Get left and right joystick potentiometer readings
		// Channel 1 -> Left side
		// Channel 2 -> Right side
		LeftJoystick_Raw = analog_reading(1);
		RightJoystick_Raw = analog_reading(2);
		
		LeftJoystick_Position = ADC_balance(LeftJoystick_Raw);
		RightJoystick_Position = ADC_balance(RightJoystick_Raw);
	
  }		
}

/* Initialize MISC GPIO pins */
void GPIO_initialize(void){
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
}

/* Initialize timer */
void TIM_initialize(void){
		
	// Initialize Timer 2
	timerInitStructure.TIM_Prescaler = 14400;
  timerInitStructure.TIM_CounterMode = TIM_CounterMode_Down;
  timerInitStructure.TIM_Period = 10000;
  timerInitStructure.TIM_ClockDivision = 0;
  timerInitStructure.TIM_RepetitionCounter = 0;
  TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

	// Configure so that the interrupt flag is only set upon overflow
	TIM_UpdateRequestConfig(TIM2, TIM_UpdateSource_Regular);

	// Enable the TIM2 Update Interrupt type
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	
}


/* RCC clock initialize */
void RCC_initialize(void){
	
	//GPIO A
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
  //GPIO B
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	//TIMER 2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	
	//ADC1 Clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	
}


/* Initialize nested vector interrupt control on timer 2*/
void NVIC_Configuration(void){
		
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
}


/* Initialize ADC1 */
void ADC_initialize(void){
	
	//ADCCLK = PCLK2/8 = 9MHz
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);
	
	/* Put everything back to power-on defaults */
	ADC_DeInit(ADC1);
	
	/* ADC1 and ADC2 operate independently */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	
  /* Disable the scan conversion so we do one at a time */
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	
  /* Don't do continuous conversions - do them on demand */
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	
  /* Start conversin by software, not an external trigger */
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	
  /* Conversions are 12 bit - put them in the lower 12 bits of the result */
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	
  /* Say how many channels would be used by the sequencer */
  ADC_InitStructure.ADC_NbrOfChannel = 3;

  /* Now do the setup */
  ADC_Init(ADC1, &ADC_InitStructure);
	
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */
  ADC_ResetCalibration(ADC1);
	
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));
	
  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
	
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
}

/* Read analog value */
uint16_t analog_reading(uint8_t channel){

	ADC_RegularChannelConfig(ADC1, channel, 1, ADC_SampleTime_7Cycles5);
	
  // Start the conversion
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
	
  // Wait until conversion completion
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);

	// Return analog reading
	return ADC_GetConversionValue(ADC1);	
}


/*Balance the raw joystick output with dead zone and sensitivity correction*/
uint16_t ADC_balance(uint16_t raw_value){
	
	uint16_t corrected_value = 0;
	
	if ((raw_value > (2048-deadZone))&&(raw_value < (2048+deadZone))){
		corrected_value = 2048;	
	}
	



	return corrected_value; 
}



/* Interrupt Handling */
void TIM2_IRQHandler(void)
{
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
  {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    
  }		
} 


