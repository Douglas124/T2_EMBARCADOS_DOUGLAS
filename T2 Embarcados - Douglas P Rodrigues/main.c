/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "string.h"
#include "stdio.h"
#include "math.h"

#define SDA (1<<10) //PA.10 - PLACA stm32f103
#define SCL (1<<11) //PA.11 - PLACA stm32f103
#define SDA0 GPIOA->BSRR = 1<<(10+16)    // bit set/reset register 32 bit- 16 primeiros set, 16 ultimos reset: [0000010000000000 0000000000000000]
#define SDA1 GPIOA->BSRR = SDA					 // bit set/reset register 32 bit- 16 primeiros set, 16 ultimos reset: [0000000000000000 0000010000000000]
#define SCL0 GPIOA->BSRR = 1<<(11+16)
#define SCL1 GPIOA->BSRR = SCL


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

short AC1, AC2, AC3, B1, B2, MB, MC, MD;
unsigned short AC4, AC5, AC6;
long X1, X2, B5, T, UT, UP, B6, X3, B3, B4, B7, pressao;
unsigned int conta_tempo =0, tempo_medido =0, contador =0, direcao=0, pwm = 0;
float freq_anemometro=0, quantidade_chuva=0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//--------------------------------------------------- MANDA COMANDO PARA O LCD
	void lcd_comando(uint8_t comando){
	HAL_GPIO_WritePin(GPIOA, RS_Pin,0);  //RS em zero para mandar comando
	GPIOA->BRR = 0xFF; //zera os bits de dados
	GPIOA->BSRR = comando; //seta os bits do comando
	
	HAL_GPIO_WritePin(GPIOA, EN_Pin,1);			//en 1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, EN_Pin,0);			//en 0
	HAL_Delay(1);
	}
	
//--------------------------------------------------- DESLOCA CURSOR LCD
	void lcd_GOTO(int linha, int coluna){
		if (coluna<16){
		if (linha==0)lcd_comando(0x80+coluna);
		if (linha==1)lcd_comando(0xc0+coluna);
		if (linha==2)lcd_comando(0x94+coluna);
		if (linha==3)lcd_comando(0xD4+coluna);
		}
	}
	
	
//--------------------------------------------------- MANDA DADOS PARA ESCREVER NO LCD
	void lcd_dado(char dado){
	HAL_GPIO_WritePin(GPIOA, RS_Pin,1);  //RS em zero para mandar dados
	GPIOA->BRR = 0xFF; //zera os bits de dados
	GPIOA->BSRR = dado; //seta os bits do dado
	
	HAL_GPIO_WritePin(GPIOA, EN_Pin,1);			//en 1
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, EN_Pin,0);			//en 0
	HAL_Delay(1);
	}
	

//--------------------------------------------------- ESCREVE STRINGS NO LCD
	void lcd_STRING(char vetor[]){
		int tamanho, i=0;
		tamanho = strlen(vetor);
		for(i=0;i<=tamanho;i++){
			lcd_dado(vetor[i]);
		}
	}
	

//--------------------------------------------------- INICIALIZAÇAO DO LCD	
	void lcd_init(void){
		lcd_comando(0x38);
		lcd_comando(0x0e);
		lcd_comando(0x06);
		lcd_comando(0x01);
	  HAL_Delay(100);
	}
	
//--------------------------------------------------- APAGA TUDO NO DISLAY
	void lcd_clear(){
	int i=0, j=0;
		for (i=0; i<4;i++){
			for (j=0; j<16;j++){
				lcd_GOTO(i,j);
				lcd_dado(' ');
			}
		}
	}
	
//--------------------------------------------------- START I2C PADRAO
	void start_i2c(void){
SDA1;
HAL_Delay(1);//1 milisegundo
SCL1;
HAL_Delay(1);//1 milisegundo
SDA0;
HAL_Delay(1);//1 milisegundo
SCL0;
HAL_Delay(1);//1 milisegundo
}
//--------------------------------------------------- STOP I2C PADRAO
	void stop_i2c(void) {
SDA0;
HAL_Delay(1);//1 milisegundo
SCL0;
HAL_Delay(1);//1 milisegundo
SCL1;
HAL_Delay(1);//1 milisegundo
SDA1;
HAL_Delay(1);//1 milisegundo
}

//--------------------------------------------------- Envia 1 pelo I2C
	void envia_1_i2c(void){
SDA1;
HAL_Delay(1);//1 milisegundo
SCL1;
HAL_Delay(1);//1 milisegundo
SCL0;
HAL_Delay(1);//1 milisegundo
}

//--------------------------------------------------- Envia 0 pelo I2C
	void envia_0_i2c(void){
SDA0;
HAL_Delay(1);//1 milisegundo
SCL1;
HAL_Delay(1);//1 milisegundo
SCL0;
HAL_Delay(1);//1 milisegundo
}

//--------------------------------------------------- ACK i2c
	int ack_i2c(void){
int x;																// muda a config do pino de saida para entrada para poder ler o ack depois muda para sada de novo
GPIO_InitTypeDef GPIO_InitStruct;
GPIO_InitStruct.Pin = GPIO_PIN_10; // SDA => PA.10
GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //FAZ SDA COMO ENTRADA
GPIO_InitStruct.Pull = GPIO_PULLUP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
SCL1;
HAL_Delay(1);//1 milisegundo
x = HAL_GPIO_ReadPin(GPIOA,SDA); //L^E O PINO
SCL0;
HAL_Delay(1);//1 milisegundo
GPIO_InitStruct.Pin = GPIO_PIN_10; // SDA => PA.10
GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //FAZ SDA COMO SA´IDA
GPIO_InitStruct.Pull = GPIO_PULLUP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
return x; //se 0 ok, se 1 erro
}

//--------------------------------------------------- LE UM BYTE
	int8_t le_byte(void){
	uint8_t x=0;																// muda a config do pino de saida para entrada para poder ler o ack depois muda para sada de novo
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_10; // SDA => PA.10
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT; //FAZ SDA COMO ENTRADA
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	
	for (int i=0; i<8; i++){
	SCL1;
	HAL_Delay(1);//1 milisegundo
	x = x<<1;
	x |= HAL_GPIO_ReadPin(GPIOA,SDA); //L^E O PINO
	SCL0;
	HAL_Delay(1);//1 milisegundo
	}
	

	GPIO_InitStruct.Pin = GPIO_PIN_10; // SDA => PA.10
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; //FAZ SDA COMO SA´IDA
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	return x; 

}
	
//--------------------------------------------------- ENVIA UM BYTE
	void envia_byte(uint8_t dado){
		if ((dado & 0x80)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x40)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x20)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x10)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x08)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x04)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x02)==0) envia_0_i2c();
			else envia_1_i2c();
		if ((dado & 0x01)==0) envia_0_i2c();
			else envia_1_i2c();
	}
//--------------------------------------------------- CALIBRA SENSOR BMP180
	void le_calib_bmp180(void){
		uint8_t dado1, dado2;
	
//----------------------------- AC1		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xAA);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xAB);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		AC1 = (dado1 << 8) + dado2;
			

//----------------------------- AC2	
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xAC);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xAD);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		AC2 = (dado1 << 8) + dado2;
			
//----------------------------- AC3	
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xAE);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xAF);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		AC3 = (dado1 << 8) + dado2;
			

//----------------------------- AC4	
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB0);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB1);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		AC4 = (dado1 << 8) + dado2;
				
//----------------------------- AC5		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB2);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB3);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		AC5 = (dado1 << 8) + dado2;
			

//----------------------------- AC6	
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB4);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB5);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		AC6 = (dado1 << 8) + dado2;
			
//----------------------------- B1	
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB6);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB7);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		B1 = (dado1 << 8) + dado2;
			

//----------------------------- B2
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB8);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xB9);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		B2 = (dado1 << 8) + dado2;
			
//----------------------------- MB	
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xBA);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xBB);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		MB = (dado1 << 8) + dado2;
			

//----------------------------- MC
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xBC);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xBD);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		MC = (dado1 << 8) + dado2;
			
//----------------------------- MD
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xBE);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado1 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		start_i2c ();
		envia_byte(0xEE);
		ack_i2c();
		envia_byte(0xBF);
		ack_i2c();
		start_i2c();
		envia_byte(0xEF);	
		ack_i2c();
		dado2 = le_byte();
		envia_1_i2c();
		stop_i2c();
		
		MD = (dado1 << 8) + dado2;

	}
	
//--------------------------------------------------- LE TEMPERATURA SENSOR BMP180	
long le_temp_bmp180(void){
	uint8_t dado1, dado2;
	
	start_i2c();
	envia_byte(0xEE);
	ack_i2c();
	envia_byte(0xF4);
	ack_i2c();
	envia_byte(0x2E);
	ack_i2c();
	stop_i2c();
	HAL_Delay(5);
	
	start_i2c(); 
	envia_byte(0xEE);
	ack_i2c();
	envia_byte(0xF6);
	ack_i2c();
	start_i2c();
	envia_byte(0xEF);
	ack_i2c();
	dado1 = le_byte();
	envia_0_i2c();
	dado2 = le_byte();
	envia_1_i2c();
	stop_i2c();
	
	UT = (dado1 << 8) + dado2;
	X1 = (UT - AC6)*AC5/32768;
	X2 = MC*2048/(X1+MD);
	B5 = X1 + X2;
	T = (B5+8)/16;
	
	return (int)T/10;
}	

//--------------------------------------------------- LE PRESSAO SENSOR BMP180	
long le_press_bmp180(void){
	uint8_t dado1, dado2, dado3;
	
	start_i2c();
	envia_byte(0xEE);
	ack_i2c();
	envia_byte(0xF4);
	ack_i2c();
	envia_byte(0x34 +(0<<6));
	ack_i2c();
	stop_i2c();
	HAL_Delay(5);
	
	start_i2c(); 
	envia_byte(0xEE);
	ack_i2c();
	envia_byte(0xF6);
	ack_i2c();
	start_i2c();
	envia_byte(0xEF);
	ack_i2c();
	dado1 = le_byte();
	envia_0_i2c();
	dado2 = le_byte();
	envia_0_i2c();
	dado3 = le_byte();
	envia_1_i2c();
	stop_i2c();
	

	UP = ((dado1 << 16) + (dado2<<8) + dado1)>>8;
	X1 = (UT - AC6)*AC5/32768;
	X2 = MC*2048/(X1+MD);
	B5 = X1 + X2;
	B6 = B5 - 4000;
	X1 = (B2*(B6*B6/4096))/2048;
	X2 = AC2*B6/2048;
	X3 = X1 + X2;
	B3 = (((AC1*4+X3))+2)/4;
//	X1 = AC3* B6/8192;
//	X2 = (B1*(B6*B6/4096))/65536;
//	X3 = ((X1 + X2)+2)/4;
	B4 = AC4 * (unsigned long)(X3+32768)/32768;
	B7 = ((unsigned long)UP-B3)*(50000);
	if (B7 < 0x80000000) pressao = (B7*2)/B4;
		else pressao = (B7/B4)*2;
	X1 = (pressao/256)*(pressao/256);
	X1 = (X1*3038)/65536;
	X2 = (-7357*pressao)/65536;
	pressao = pressao + (X1 + X2 + 3791)/16;
	
	
	return (int)pressao/100;
}	
//--------------------------------------------------- CALCULO ALTITUDE
float altitude(void){
	int pressao;
	float altitude=0;
	float aux1 = 1.0/5255.0; 
	pressao = le_press_bmp180();
	altitude = 44330.0 * (1-pow((pressao/1023.25), aux1));
	return (int)altitude;
}

//--------------------------------------------------- LEITURA DO AD
uint16_t le_AD(int escolha){
	uint16_t pot_motor, vento, luminosidade;

//-------------------------------------- leitura vento	
	HAL_GPIO_WritePin(GPIOB, MUX_A_Pin,0);
	HAL_GPIO_WritePin(GPIOB, MUX_B_Pin,0);
	HAL_Delay(10);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,100);
	vento = HAL_ADC_GetValue(&hadc1); 
	HAL_ADC_Stop(&hadc1);
	
//-------------------------------------- leitura luminosidade
	HAL_GPIO_WritePin(GPIOB, MUX_A_Pin,1);
	HAL_GPIO_WritePin(GPIOB, MUX_B_Pin,0);
	HAL_Delay(10);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,100);
	luminosidade = HAL_ADC_GetValue(&hadc1); 
	HAL_ADC_Stop(&hadc1);
	
//-------------------------------------- leitura pot motor
	HAL_GPIO_WritePin(GPIOB, MUX_A_Pin,0);
	HAL_GPIO_WritePin(GPIOB, MUX_B_Pin,1);
	HAL_Delay(10);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,100);
	pot_motor = HAL_ADC_GetValue(&hadc1); 
	HAL_ADC_Stop(&hadc1);
	
  switch (escolha){
		case 1:
		return vento;
		break;
		case 2:
		return luminosidade;
		break;
		case 3:
		return pot_motor;
		break;
	}
}

//--------------------------------------------------- INTERRUPÇÂO PARA PWM
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	contador ++;
	conta_tempo ++;
	if (pwm == 0) HAL_GPIO_WritePin(GPIOB, PWM1_Pin,0);
	else{
		if (contador <= pwm)	HAL_GPIO_WritePin(GPIOB, PWM1_Pin,1);
		else 	HAL_GPIO_WritePin(GPIOB, PWM1_Pin,0);	
		if (contador >= 100) {
		contador = 0;
		HAL_GPIO_WritePin(GPIOB, PWM1_Pin,1);
		}
	}
		
}

//--------------------------------------------------- CONTROLE MOTOR
void controle_motor(void){
	uint16_t leitura_AD = 0;
	leitura_AD = le_AD(3);

	if (leitura_AD > 2035-50 && leitura_AD < 2035+50){
	pwm = 0;
	direcao = 3;
	}
	else{
	if (leitura_AD <= 2035-50){
	pwm =((leitura_AD*100)/2035);
	HAL_GPIO_WritePin(GPIOB, RL1_Pin,1);
	direcao =1;
	}
	
	if (leitura_AD >= 2035+50){
	pwm =((leitura_AD - 2035+50)*100)/2035;
	HAL_GPIO_WritePin(GPIOB, RL1_Pin,0);
	direcao = 2;
	}
}
}


//--------------------------------------------------- LEITURA VENTO
int dir_vento (void){
	uint16_t vento;
	int direcao=0; // N=1, NE=2, E=3, ES=4, S=5, SW=6, W=7, NW=8
	vento = le_AD(1);
	
	if (vento<= 3554+50 && vento>=3554-50) direcao = 0;
	else if (vento<= 2583+50 && vento>=2583-50) direcao = 45;
	else if (vento<= 713+50 && vento>=713-50) direcao = 90;
	else if (vento<= 1291+50 && vento>=1291-50) direcao = 135;
	else if (vento<= 1845+50 && vento>=1845-50) direcao = 180;
	else if (vento<= 3136+50 && vento>=3136-50) direcao = 225;
	else if (vento<= 3911+50 && vento>=3911-50) direcao = 270;
	else if (vento<= 3788+50 && vento>=3788-50) direcao = 315;
	else direcao =0;
	
	return direcao;
}

//--------------------------------------------------- LEITURA LUMINOSIDADE
int lumi_dia (void){
	uint16_t luminosidade;
	int estado_dia=0; // sol=1, parcialmente nublado=2, nublado=3, anoitecer=4, noite=5
	luminosidade = le_AD(2);
	
	if (luminosidade>=4096-50) estado_dia = 1;
	else if (luminosidade< 4096-50 && luminosidade>=2337) estado_dia = 2;
	else if (luminosidade< 2337 && luminosidade>=61) estado_dia = 3;
	else if (luminosidade< 61 && luminosidade>=8) estado_dia = 4;
	else if (luminosidade< 8) estado_dia = 5;
	else luminosidade =0;
	
	return estado_dia;
}

//--------------------------------------------------- INTERRUPÇÃO SENSOR CHUVA
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	static int aux_contador =0, diferenca =0, aux=0;
	if (GPIO_Pin == GPIO_PIN_12){
	quantidade_chuva = quantidade_chuva + 1;
	}
	
	if (GPIO_Pin == GPIO_PIN_15){
		tempo_medido = conta_tempo;
		conta_tempo = 0;
	}
	
}

	
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char vetor[30], dados_serial[100];
	

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
		HAL_TIM_Base_Start_IT(&htim1);
		le_calib_bmp180();
		lcd_init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	
		sprintf((char*)dados_serial, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",(int)le_temp_bmp180(), (int)le_press_bmp180(), (int)altitude(),pwm,direcao,dir_vento(),(int)quantidade_chuva,lumi_dia(),(int)(1/(tempo_medido*0.0001)));
		HAL_UART_Transmit(&huart1, dados_serial, strlen((char*)dados_serial), 500);
		//HAL_Delay(1000);
		
		controle_motor();	
		sprintf(vetor, "T:%doC P:%3dPa A:%dm",(int)le_temp_bmp180(), (int)le_press_bmp180(), (int)altitude());
		lcd_GOTO(0,0);
		lcd_STRING(vetor);
		
		if (direcao == 1){
		sprintf(vetor, "VM:%d%%D Vento:%do",pwm, dir_vento());
		lcd_GOTO(1,0);
		lcd_STRING(vetor);
		}
		else if (direcao ==2 ) {
		sprintf(vetor, "VM:%.2d%%E Vento:%do",(int)pwm, dir_vento());
		lcd_GOTO(1,0);
		lcd_STRING(vetor);
		}
		else if (direcao ==3 ) {
		sprintf(vetor, "VM:Parado Vento:%do", dir_vento());
		lcd_GOTO(1,0);
		lcd_STRING(vetor);
		}
		
		if (lumi_dia() == 1){
		sprintf(vetor, "Chuva:%dmm  SOL     ",(int)quantidade_chuva);
		lcd_GOTO(2,0);
		lcd_STRING(vetor);
		}
		else if (lumi_dia() == 2){
		sprintf(vetor, "Chuva:%dmm  PAR NUB",(int)quantidade_chuva);
		lcd_GOTO(2,0);
		lcd_STRING(vetor);
		}
		else if (lumi_dia() == 3){
		sprintf(vetor, "Chuva:%dmm  NUBLADO    ",(int)quantidade_chuva);
		lcd_GOTO(2,0);
		lcd_STRING(vetor);
		}
		else if (lumi_dia() == 4){
		sprintf(vetor, "Chuva:%dmm  ANOITE",(int)quantidade_chuva);
		lcd_GOTO(2,0);
		lcd_STRING(vetor);
		}
		else if (lumi_dia() == 5){
		sprintf(vetor, "Chuva:%dmm  NOITE       ",(int)quantidade_chuva);
		lcd_GOTO(2,0);
		lcd_STRING(vetor);
		}

		
		sprintf(vetor, "Vel Vento:%2dkm/h",(int)(1/(tempo_medido*0.0001)));
		lcd_GOTO(3,0);
		lcd_STRING(vetor);
		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
