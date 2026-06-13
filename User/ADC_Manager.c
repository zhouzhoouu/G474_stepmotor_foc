#include "ADC_Manager.h"
#include "Control_Loop.h"
#include "adc.h"
#include "ma600a.h"
#include "spi.h"

short raw_theta_fix[100] =
        {-606, -592, -611, -612, -595, -590, -611, -597, -581, -614,
         -597, -584, -599, -601, -587, -584, -607, -591, -574,
         -606, -592, -580, -600, -600, -586, -583, -604, -591,
         -576, -610, -595, -582, -600, -601, -587, -589, -610,
         -596, -581, -613, -600, -587, -612, -613, -596, -594,
         -615, -603, -590, -624, -610, -596, -619, -617, -604,
         -608, -628, -615, -599, -632, -619, -607, -631, -629,
         -615, -616, -637, -623, -609, -642, -627, -612, -637,
         -633, -620, -623, -640, -625, -611, -643, -629, -615,
         -639, -631, -617, -622, -638, -625, -610, -642, -626,
         -611, -635, -631, -617, -621, -636, -622, -608, -640
        };
short raw_theta_fix_bias = 328;

#define N_CH 2

static volatile uint32_t ADC_Buffer[N_CH*2];
static volatile ADC_Raw_data ADC_raw_read;
static volatile ADC_Angle_data ADC_angle_tmp;

void ADC_Manager_Init(void){

    ma600a_init();
    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

    HAL_Delay(80);

    HAL_ADC_Start(&hadc2);
    HAL_ADCEx_MultiModeStart_DMA(&hadc1,(uint32_t*)ADC_Buffer,N_CH*2);

    Control_Manager_Init();

}

void ADC_Extract(volatile uint32_t* pdata){

    ADC_raw_read.Voltage_Bus = (int)(pdata[1]&0x0000ffff);
//    ADC_raw_read.Voltage_B = (int)(pdata[0]>>16) - 2048;

//    ADC_raw_read.Voltage_A = (int)(pdata[1]&0x0000ffff) - 2048;
//    ADC_raw_read.Voltage_C = (int)(pdata[1]>>16) - 2048;

    ADC_raw_read.Current_A = -((int)(pdata[0]&0x0000ffff) - 2048);
    ADC_raw_read.Current_B = (int)(pdata[0]>>16) - 2048;

}


void ADC_Angle_Extract(void){

    static uint8_t mag_txbuf[4]={0, 0, 0, 0};
    uint8_t rx_data[4] = {0};

    HAL_GPIO_WritePin(CSn_GPIO_Port,CSn_Pin,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, mag_txbuf, rx_data, 4, 0xFFFF);
    HAL_GPIO_WritePin(CSn_GPIO_Port,CSn_Pin,GPIO_PIN_SET);

    uint16_t raw_angle = ((uint16_t)rx_data[0]<<8) | rx_data[1];
    uint16_t raw_omega_tmp = ((uint16_t)rx_data[2]<<9) | ((uint16_t)rx_data[3]<<1) | (rx_data[2]>>7);
    short raw_omega = (short)raw_omega_tmp;
//    raw_omega /= 2;

#ifdef INVERSE_ENCODER
    raw_angle = MAX_ENCODER_ANGLE - raw_angle;
#endif

#ifdef INVERSE_ENCODER_SPEED
    raw_omega *= -1;
#endif


#ifndef ENCODER_CALIBRATION
    uint32_t n_compensation = ((uint32_t)raw_angle * 100) / MAX_ENCODER_ANGLE;
    int n_tail = raw_angle*100 - (int)n_compensation*MAX_ENCODER_ANGLE;

    int f1 = raw_theta_fix[n_compensation];
    int f2 = 0;
    if (n_compensation < 99) {
        f2 = raw_theta_fix[n_compensation + 1];
    } else {
        f2 = raw_theta_fix[0];
    }
    float k = (float)n_tail*(1.f/MAX_ENCODER_ANGLE);
    raw_angle += f1 + (int)k*(f2-f1) + raw_theta_fix_bias; // 插值补偿
#endif

    ADC_angle_tmp.angle_raw = raw_angle;
    ADC_angle_tmp.omega_mec = (float)raw_omega*5.722f*(1/60.f)*2*PI;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
    if (hadc->Instance==hadc1.Instance){
        LL_GPIO_SetOutputPin(LD1_GPIO_Port,LD1_Pin);
        ADC_Extract(ADC_Buffer + N_CH);
        Control_Manager_Loop((ADC_Raw_data*)&ADC_raw_read, (ADC_Angle_data*)&ADC_angle_tmp);
        LL_GPIO_ResetOutputPin(LD1_GPIO_Port,LD1_Pin);
    }

}
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc){

    if (hadc->Instance==hadc1.Instance){
        LL_GPIO_SetOutputPin(LD1_GPIO_Port,LD1_Pin);
        ADC_Extract(ADC_Buffer);
        Control_Manager_Loop((ADC_Raw_data*)&ADC_raw_read, (ADC_Angle_data*)&ADC_angle_tmp);
        LL_GPIO_ResetOutputPin(LD1_GPIO_Port,LD1_Pin);
    }
}