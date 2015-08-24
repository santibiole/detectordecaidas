#include <stdint.h>
#include "stm32l1xx.h"			// Header del micro
#include "stm32l1xx_rcc.h"		// Para configurar el (Reset and clock controller)
#include "stm32l1xx_gpio.h"		// Perifericos de E/S
#include "stm32l1xx_i2c.h"		// Inter Integrated Circuit
#include "stm32l1xx_tim.h"		// Modulos Timers
#include "stm32l1xx_syscfg.h"	// configuraciones Generales
#include "misc.h"				// Vectores de interrupciones (NVIC)
#include "bsp.h"
#include "MMA8451.h"

#define LED_A GPIO_Pin_6
#define LED_R GPIO_Pin_7

#define BOTON_EP GPIO_Pin_4
#define BOTON_EM GPIO_Pin_5

/* Leds disponibles */
const uint16_t leds[] = {LED_A,LED_R};
GPIO_TypeDef* leds_port[] = {GPIOA,GPIOA};

/* Leds disponibles */
const uint16_t botones[] = {BOTON_EP,BOTON_EM};
GPIO_TypeDef* botones_port[] = {GPIOA,GPIOA};


/*
	Prototipo de una función externa, es decir, una frecuencia que va a estar implementada en algún otro
	lugar de nuestro proyecto. El linker es el que se va a encargar de ubicar donde está implementada.
 */
extern void APP_ISR_1ms (void);
extern void APP_ISR_AF (void);


volatile uint16_t bsp_count_ms = 0; // Defino como volatile para que el compilador no interprete el while(bsp_count_ms) como un bucle infinito.


void led_on(uint8_t led) {
	GPIO_SetBits(leds_port[led], leds[led]);
}

void led_off(uint8_t led) {
	GPIO_ResetBits(leds_port[led], leds[led]);
}

void led_toggle(uint8_t led) {
	GPIO_ToggleBits(GPIOA, leds[led]);
}

uint8_t sw_getState(uint16_t boton) {
	return GPIO_ReadInputDataBit(botones_port[boton], botones[boton]);
}

void bsp_delay_ms(uint16_t x){
	bsp_count_ms = x;
	while (bsp_count_ms);
}

/**
 * @brief Interrupción llamada al pasar 1ms
 */
void TIM2_IRQHandler(void) {

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		// Rutina:
		APP_ISR_1ms();
		if (bsp_count_ms) { //Pregunto si bsp_count_ms es distinto de 0.
			bsp_count_ms--;
		}
	}
}

void I2C1_ER_IRQHandler(void) {
	if(I2C_GetITStatus(I2C1, I2C_IT_AF) != RESET){
		I2C_ClearITPendingBit(I2C1, I2C_IT_AF);
		APP_ISR_AF();
	}
}

/* Declaración de funciones */
void bsp_boot_init();
void bsp_i2c_init();
void bsp_led_init();
void bsp_sa0_init();
void bsp_sw_init();
void bsp_timer_init();

void bsp_init() {
	bsp_boot_init();
	bsp_i2c_init();
	bsp_led_init();
	bsp_sw_init();
	bsp_timer_init();
}

void bsp_boot_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Habilitación del clock del periferico*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/*Configure GPIO pin : PA */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*
 * @brief Inicialización del puerto I2C1
 */
void bsp_i2c_init() {
	I2C_DeInit(I2C1);

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Habilitación del clock del periferico*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	//RCC_PCLK1Config(RCC_HCLK_Div16);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_ResetBits(GPIOB, GPIO_Pin_5);

	I2C_InitStruct.I2C_ClockSpeed = 100000;
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStruct.I2C_OwnAddress1 = 0x0A;
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_Init(I2C1, &I2C_InitStruct);

	I2C_ITConfig(I2C1, I2C_IT_ERR , ENABLE);
//	I2C_ITConfig(I2C1, I2C_IT_EVT , ENABLE);

	I2C_Cmd(I2C1, ENABLE);

	/* Configure the Priority Group to 1 bit */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* Configure the I2C event priority */
	NVIC_InitStructure.NVIC_IRQChannel = I2C1_ER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Configure I2C error interrupt to have the higher priority */
//	NVIC_InitStructure.NVIC_IRQChannel = I2C1_EV_IRQn;
//	NVIC_Init(&NVIC_InitStructure);
}

/*
 * @brief Inicialización de leds
 */
void bsp_led_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Habilitación del clock del periferico*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/*Configure GPIO pin : PA */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void bsp_sa0_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Habilitación del clock del periferico*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

	/*Configure GPIO pin : PA */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*
 * @brief Inicialización de switches
 */
void bsp_sw_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Habilitación del clock del periferico*/
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/*
 * @brief Inicialización del timer 2
 */
void bsp_timer_init(void) {
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Habilito la interrupcion global del  TIM2 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Habilitación del clock del periferico*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* Configuracion de la base de tiempo */
	TIM_TimeBaseStruct.TIM_Period = 1000; // 1 MHz bajado a 1 KHz (1 ms). ¿Cómo? Cuento 1us, aumento el contador, y cuando llego a mil tengo 1ms, es decir 1KHz.
	TIM_TimeBaseStruct.TIM_Prescaler = (2 * 8000000 / 1000000) - 1; // 8 MHz bajado a 1 MHz - Pre Escalador.
	TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1; // Divisor.
	TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up; // Como queremos que cuente.
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStruct); // Inicializamos timer.
	/* TIM habilitado */
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); // Inicializamos la interrupción.
	/* TIM2 contador habilitado */
	TIM_Cmd(TIM2, ENABLE);
}
