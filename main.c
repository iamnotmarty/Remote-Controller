/* Imported files */
#include <stdio.h>
#include <stdbool.h>
#include "stdlib.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/* STM32F103 SPI2 output pin definition */
#define SPI_GPIO_Port	GPIOB
#define SPI_CSN				GPIO_Pin_12
#define SPI_SCK 			GPIO_Pin_13
#define SPI_MISO 			GPIO_Pin_14
#define SPI_MOSI 			GPIO_Pin_15
#define SPI_CE 				GPIO_Pin_10

/* Analog input port and pin definition*/
#define joystick_port_L	GPIOA
#define joystick_port_R GPIOA
#define joystick_pin_L	GPIO_Pin_1
#define joystick_pin_R	GPIO_Pin_2

/* Define static coefficients */
static uint16_t const deadZone = 400;
static uint8_t const sensitivity = 5;
static uint8_t const starting_percentage = 50;
static uint16_t const max_resolution = 4095;
static uint16_t const half_max_resolution = 2048;
static uint16_t const resolution_upper = 3995;
static uint16_t const resolution_lower = 100; 

/* Define static variables */
static uint16_t deadZone_Upper = 0;
static uint16_t deadZone_Lower = 0;
static uint8_t active_range = 0;
static uint16_t step_per_portion = 0;
static uint16_t LeftJoystick_Raw = 0;
static uint16_t RightJoystick_Raw = 0;
static int8_t LeftJoystick_Position = 0;
static int8_t RightJoystick_Position = 0;

/* Microcontroller one-time setup function declaration */
static void GPIO_initialize(void);
static void TIM_initialize(void);
static void NVIC_Configuration(void);
static void RCC_initialize(void);
static void ADC_initialize(void);

/* User function declaration */
static uint16_t analog_reading(uint8_t);
int8_t ADC_balance(uint16_t);
void deadZone_check(void);

/* Type Define */
GPIO_InitTypeDef  GPIO_InitStructure;
TIM_TimeBaseInitTypeDef timerInitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_OCInitTypeDef outputChannelInit = {0,};	
ADC_InitTypeDef  ADC_InitStructure;

int main(void) {
	
	RCC_initialize();
	GPIO_initialize();
	NVIC_Configuration();	
	TIM_initialize();
	ADC_initialize();
	
	deadZone_Upper = half_max_resolution+deadZone;
	deadZone_Lower = half_max_resolution-deadZone;
	active_range = 100 - starting_percentage;
	step_per_portion = (max_resolution - deadZone_Upper)/active_range;
	
  // Start Timer 2 interrupt
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);

	while(1) {
		
		// Get left and right joystick potentiometer readings
		// Channel 1 -> Left side
		// Channel 2 -> Right side
		LeftJoystick_Raw = analog_reading(1);
		RightJoystick_Raw = analog_reading(2);
		
		// Map left and right potentiometer raw data
		LeftJoystick_Position = ADC_balance(LeftJoystick_Raw);
		RightJoystick_Position = ADC_balance(RightJoystick_Raw);
		
		// Joystick value difference sensitivity balance
		if ( abs(LeftJoystick_Position - RightJoystick_Position) <= sensitivity){
			LeftJoystick_Position = RightJoystick_Position;				
		} else;
	
		// Send values over wireless transceiver
		
  }		
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

/* Initialize GPIO pins */
void GPIO_initialize(void){
	
	// Configure left joystick Y axis analog input
	GPIO_InitStructure.GPIO_Pin = joystick_pin_L;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(joystick_port_L, &GPIO_InitStructure);
	
	// Configure right joystick Y axis analog input
	GPIO_InitStructure.GPIO_Pin = joystick_pin_R;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;	
	GPIO_Init(joystick_port_R, &GPIO_InitStructure);
	
}


/* Initialize nested vector interrupt control on timer 2*/
void NVIC_Configuration(void){
		
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;      
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
	
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


/* Map raw potentiometer value to -10 - +10 range

Input: uint16_t (raw potentiometer input from 0-4,095)
Output: int8_t (balanced value from -100 to +100)

*/

int8_t ADC_balance(uint16_t raw_value){
	
	int8_t corrected_value = 0;
	uint8_t steps = 0;

// Raw value between dead zone	
	if ((raw_value >= deadZone_Lower) && (raw_value <= deadZone_Upper)){
		corrected_value = 0;	

		// Raw value above dead zone upper limit and below max resolution
	} else if ((raw_value > deadZone_Upper) && (raw_value <= resolution_upper)){
		
		steps = (raw_value - deadZone_Upper)/step_per_portion;
				
		corrected_value = starting_percentage + steps;
		
	// Raw value below dead zone lower limit and above zero
	} else if ((raw_value < deadZone_Lower) && (raw_value >= resolution_lower)){
		
		steps = ((max_resolution-raw_value)-deadZone_Upper)/step_per_portion;
		
		corrected_value = (starting_percentage + steps)*(-1);
				
	// Raw value near 0
	} else if ((raw_value < resolution_lower) && (raw_value >= 0)){	
		corrected_value = -100;	
	
	// Raw value near max resolution
	} else if ((raw_value > resolution_upper) && (raw_value <= max_resolution)){
		corrected_value = 100;
	
	// For all other raw values
	} else {
		corrected_value = 0;
	
	}
	
	// Return the corrected value
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


