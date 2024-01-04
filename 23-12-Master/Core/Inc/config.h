#ifndef __CONFIG_H
#define __CONFIG_H
#include <stdint.h>
#include <string.h>
#include "main.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

#define spi_enable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET);
#define spi_disable HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET);
#define S2_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
#define S2_H HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#define S3_L HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
#define S3_H HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET);

extern DMA_HandleTypeDef hdma_adc1;

//Read color
uint16_t tick = 0; // Thoi gian doc RGB
uint16_t timeout = 0; // thoi gian thoat khoi doc mau

//tcrt5000 value
uint16_t u16_ADCScanVal[5],ADCValue[5],calib[5];
uint8_t digital[5],pre_digital[5]; 
uint16_t TONG = 0; 
uint16_t TH ;
uint16_t Color = 0;
uint16_t Mau;
uint8_t i; //bien dem vi tri sensor
float sum=0; //tong gia tri adc
uint16_t threshold=3000; //nguong doc line den
uint8_t limit_pos=34; //gioi han sai so vi tri
uint8_t limit_left=0,limit_right=0; //gioi han cam bien do line

//input value
float vR=400; //van toc dai mong muon (mm/s)
float wR; //van toc goc mong muon (rad/s)
float tsamp=0.01; //thoi gian lay mau (s)

//PID
float kp=0.0504;
float ki=0.1;
float kd=0.000229; 
float b=195; //khoang cach tam 2 banh (mm)
float D=85; //duong kinh banh xe (mm)
float L=180; //khoang cach tu truc dong co den mat cam bien
uint16_t ymax=3230;//gia tri lon nhat cam bien tren line den
uint16_t ymin=230; //gia tri nho nhat cam bien tren nen trang

//calculate value
float pre_e2=0,e2=0; //sai so e2
float inte2=0; //khau tich phan
float vleft=0,vright=0,wleft=0,wright=0; //van toc banh trai va banh phai
float v=0,w=0; //van toc dai va van toc goc

//position value
float weight; //trong so
float slope=0.3959; //do doc gia tri
float intercept=-0.7642; //giao diem truc hoanh
float pos=0; //gia tri vi tri tinh toan

//transmit value
uint8_t txBuffer[4] ; //mang truyen du lieu
uint8_t dem=0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);

void SystemClock_Config(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
void Error_Handler(void);

void MX_ADC1_Init(void);
void MX_SPI1_Init(void);
void MX_TIM2_Init(void);
void MX_DMA_Init(void);
void MX_GPIO_Init(void);
void Error_Handler(void);
#endif

