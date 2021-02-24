#include "stm32f4xx.h"
#include "stm32f401_discovery.h"
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_mpu6050.h"
#include "math.h"
/* Private macro */
/* Private variables */
uint8_t flag;
typedef float floatType;
int duty;
floatType diff, ref, pro, integ, error, pError, Kp, Kd, Ki, Acc, Gyro, Angle;
TM_MPU6050_Custom_t data;
/* Private function prototypes */
void forward(uint16_t duty_value);
void reverse(uint16_t duty_value);
void stop();
double abs_val(double value);
/* Private functions */

/**
 **===========================================================================
 **
 **  Abstract: main program
 **
 **===========================================================================
 */
int main(void) {

	double PDout = 0;
	duty = 0;
	ref = 0;
	flag = 0;
	error = 0, pError = 0;
	Acc = 0;
	Gyro = 0;
	pro = 0;
	diff = 0;
	integ = 0;
	Angle = 0;
	Kp = 5;
	Kd = 80;
	Ki = 0.005;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	TM_DELAY_Init();
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);

	TM_MPU6050_Init_Custom(&data, TM_MPU6050_Device_0,
			TM_MPU6050_Accelerometer_4G, TM_MPU6050_Gyroscope_500s);
	BK_MPU6050_Activate_DLPF_Custom(&data, BK_MPU6050_BW_Acc_21_Gyro_20);

	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin = (GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &gpio);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM4);

	TIM_TimeBaseInitTypeDef timStruct;
	timStruct.TIM_Period = 8399;
	timStruct.TIM_Prescaler = 0;
	timStruct.TIM_ClockDivision = 0;
	timStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &timStruct);

	TIM_OCInitTypeDef oc;
	oc.TIM_OCMode = TIM_OCMode_PWM1;
	oc.TIM_OCPolarity = TIM_OCPolarity_High;
	oc.TIM_OutputState = TIM_OutputState_Enable;
	oc.TIM_Pulse = 0;

	TIM_OC1Init(TIM4, &oc);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC2Init(TIM4, &oc);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC3Init(TIM4, &oc);
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_OC4Init(TIM4, &oc);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM4, ENABLE);
	TIM_Cmd(TIM4, ENABLE);

	while (1) {
		Delayms(1);
		STM_EVAL_LEDToggle(LED5);
		TM_MPU6050_ReadAll_Custom(&data);
		Acc = -57.296 * (data.Accelerometer_X - 170) * data.Acce_Mult;
		Gyro = (data.Gyroscope_Y - 60) * data.Gyro_Mult;
		if (!flag) {
			Angle = Acc;
			flag = 1;
		} else
			Angle = 0.9987 * (Angle + Gyro * 0.001) + 0.0013 * Acc;

		error = ref - Angle;
		pro = Kp * error;
		diff = Kd * (error - pError);
		integ += Ki * error;
		PDout = pro + diff + integ;
		pError = error;
		duty = (int) (2 * PDout * 83.99);
		if (duty == 0)
			stop();
		else if (duty > 0)
			reverse(duty + 1000);
		else
			forward(-duty + 1000);

	}
}

void forward(uint16_t duty_value) {
	STM_EVAL_LEDToggle(LED6);
	TIM4->CCR2 = 0;
	TIM4->CCR4 = 0;
	TIM4->CCR1 = duty_value;
	TIM4->CCR3 = duty_value;
}
void reverse(uint16_t duty_value) {
	STM_EVAL_LEDToggle(LED3);
	TIM4->CCR1 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR2 = duty_value;
	TIM4->CCR4 = duty_value;
}
void stop() {
	TIM4->CCR1 = 0;
	TIM4->CCR3 = 0;
	TIM4->CCR2 = 0;
	TIM4->CCR4 = 0;
}
double abs_val(double value) {
	if (value >= 0)
		return value;
	else
		return (-value);
}

/*
 * Callback used by stm32f401_discovery_audio_codec.c.
 * Refer to stm32f401_discovery_audio_codec.h for more info.
 */
void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size) {
	/* TODO, implement your code here */
	return;
}

/*
 * Callback used by stm32f401_discovery_audio_codec.c.
 * Refer to stm32f401_discovery_audio_codec.h for more info.
 */
uint16_t EVAL_AUDIO_GetSampleCallBack(void) {
	/* TODO, implement your code here */
	return -1;
}
