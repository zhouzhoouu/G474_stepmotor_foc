#include "ADC_Manager.h"
#include "Control_Loop.h"
#include "adc.h"
#include "spi.h"

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

}

void ADC_Extract(volatile uint32_t* pdata){

    ADC_raw_read.Voltage_Bus = (int)(pdata[1]&0x0000ffff);
//    ADC_raw_read.Voltage_B = (int)(pdata[0]>>16) - 2048;

//    ADC_raw_read.Voltage_A = (int)(pdata[1]&0x0000ffff) - 2048;
//    ADC_raw_read.Voltage_C = (int)(pdata[1]>>16) - 2048;

    ADC_raw_read.Current_A = (int)(pdata[0]&0x0000ffff) - 2048;
    ADC_raw_read.Current_B = (int)(pdata[0]>>16) - 2048;

}


void ADC_Angle_Extract(void){

    static const uint16_t cmd_addre = (0b1010<<12)|0x003;
    static uint8_t mag_txbuf[5]={cmd_addre>>8, cmd_addre&0xFF,0,0, 0};
    uint8_t rx_data[5] = {0};

    HAL_GPIO_WritePin(CSn_GPIO_Port,CSn_Pin,GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, mag_txbuf, rx_data, 5, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CSn_GPIO_Port,CSn_Pin,GPIO_PIN_SET);

    // 组合21位原始角度数据
    // rx_data[0]对应0x003, rx_data[1]对应0x004, rx_data[2]对应0x005, rx_data[3]对应0x006
    uint32_t raw_angle = ((uint32_t)rx_data[2] << 13) | ((uint32_t)rx_data[3] << 5) | (rx_data[4] >> 3);


    ADC_angle_tmp.angle_raw = raw_angle;
//    ADC_angle_tmp.lost_mag = (mag_rxbuf[3]&0b00000010)>>1;
//    ADC_angle_tmp.overspeed = (mag_rxbuf[5]&0b00001000)>>3;
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