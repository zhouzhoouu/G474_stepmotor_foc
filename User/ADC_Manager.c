#include "ADC_Manager.h"
#include "Control_Loop.h"
#include "adc.h"
#include "spi.h"
#include "arm_math.h"

short raw_theta_fix[100] = {
    547, 848, 971, 1133, 1262, 1426, 1501, 1650, 1699, 1775,
    1744, 1850, 1832, 1810, 1800, 1663, 1518, 1395, 1263, 1136,
    1037, 964, 839, 806, 715, 711, 700, 579, 421, 261,
    102, -110, -295, -502, -738, -899, -1117, -1164, -1253, -1268,
    -1358, -1414, -1428, -1404, -1397, -1318, -1313, -1158, -1056, -956,
    -714, -491, -384, -229, -117, -15, 162, 240, 337, 415,
    488, 505, 596, 606, 654, 554, 484, 413, 333, 277,
    277, 194, 207, 178, 268, 270, 370, 290, 266, 164,
    91, -36, -138, -317, -407, -530, -619, -701, -633, -618,
    -653, -641, -567, -508, -381, -306, -156, -10, 124, 349
};

#define N_CH 2

static volatile uint32_t ADC_Buffer[N_CH*2];
static volatile ADC_Raw_data ADC_raw_read;
static volatile ADC_Angle_data ADC_angle_tmp;

void ADC_Manager_Init(void){

    HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);

    HAL_Delay(50);

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

    static uint64_t last_tik;
    static const uint16_t cmd_addre = (0b1010<<12)|0x003;
    static uint8_t mag_txbuf[5]={cmd_addre>>8, cmd_addre&0xFF,0,0, 0};
    uint8_t rx_data[5] = {0};

    HAL_GPIO_WritePin(CSn_GPIO_Port,CSn_Pin,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, mag_txbuf, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSn_GPIO_Port,CSn_Pin,GPIO_PIN_SET);
    uint64_t tik = Get_Ctrl_Tik();
    // 组合21位原始角度数据
    // rx_data[0]对应0x003, rx_data[1]对应0x004, rx_data[2]对应0x005, rx_data[3]对应0x006
    int raw_angle = (int)(((uint32_t)rx_data[2] << 13) | ((uint32_t)rx_data[3] << 5) | (rx_data[4] >> 3));


#ifdef INVERSE_ENCODER
    raw_angle = 2097152 - raw_angle;
#endif


#ifndef ENCODER_CALIBRATION
    uint32_t n_compensation = ((uint32_t)raw_angle * 100) / 2097152;
    int n_tail = raw_angle*100 - (int)n_compensation*2097152;

    int f1 = raw_theta_fix[n_compensation];
    int f2 = 0;
    if (n_compensation < 99) {
        f2 = raw_theta_fix[n_compensation + 1];
    } else {
        f2 = raw_theta_fix[0];
    }
    float k = (float)n_tail*(1/2097152.f);
    raw_angle += f1 + (int)k*(f2-f1); // 插值补偿
#endif

    ADC_angle_tmp.angle_diff = raw_angle - ADC_angle_tmp.angle_raw;
    ADC_angle_tmp.angle_raw = raw_angle;

    while (ADC_angle_tmp.angle_diff > 1048576) { // 角度差大于180度，说明发生了跨越
        ADC_angle_tmp.angle_diff -= 2097152; // 减去360度对应的计数值
    }
    while (ADC_angle_tmp.angle_diff < -1048576) { // 角度差小于-180度，说明发生了跨越
        ADC_angle_tmp.angle_diff += 2097152; // 加上360度对应的计数值
    }

    int dtik = (int)(tik-last_tik);
    if (dtik >0)
    {
        float omega_ref = (float)ADC_angle_tmp.angle_diff * (2*PI/2097152.f)/(Ts*(float)(dtik));
        ADC_angle_tmp.omega_mec += 0.04f*(omega_ref - ADC_angle_tmp.omega_mec);
        last_tik = tik;
    }
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