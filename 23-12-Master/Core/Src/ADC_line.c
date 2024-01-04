#include "config.h"
#include "stm32f1xx.h"
#include "ADC_line.h"
#include "main.h"

void read_ADC(void)
{
	//trai qua phai: 0->4
	sum=0;
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)u16_ADCScanVal,5);	
	for (i=0;i<5;i++)
	{
		pre_digital[i]=digital[i];
		ADCValue[i]=u16_ADCScanVal[i];
		if (ADCValue[i] > threshold) digital[i]=1;
		else digital[i]=0;
	}		
	calib[0] = ymin + 0.9772*(ADCValue[0]-160) ;
	calib[1] = ymin + 0.909*(ADCValue[1]-140) ;
	calib[2] = ymin + 0.9146*(ADCValue[2]-150) ;
	calib[3] = ymin + 0.909*(ADCValue[3]-230) ;
	calib[4] = ymin + 0.9288*(ADCValue[4]-170) ;	
	sum=calib[0]+calib[1]+calib[2]+calib[3]+calib[4];
	//weight=17*(2*(calib[0]-calib[4])+(calib[1]-calib[3]))/sum; //find weight	
	//pos = (weight-intercept)/slope; // position calculate
	pos = ((-34*calib[0])+(-17*calib[1])+(0*calib[2])+(17*calib[3])+(34*calib[4]))/sum;
}

//void read_position(void)
//{
//	limit_left=0;
//	limit_right=0;
//	sum=calib[0]+calib[1]+calib[2]+calib[3]+calib[4];
//	weight=17*(2*(calib[0]-calib[4])+(calib[1]-calib[3]))/sum; //find weight	
//	//pos = (weight-intercept)/slope; // position calculate
//	//pos = ((-34*calib[0])+(-17*calib[1])+(0*calib[2])+(17*calib[3])+(34*calib[4]))/sum;
//}