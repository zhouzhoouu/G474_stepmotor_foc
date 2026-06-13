#ifndef __ADC_MANAGER_H
#define __ADC_MANAGER_H

#include "main.h"

#define MAX_ENCODER_ANGLE 65536
#define Ts (1/34000.f)
#define ADC_Basic_Gain (3.3f/4096)

#define ADC_IA_Gain (1.f/.3f)
#define ADC_IB_Gain (1.f/.3f)
#define ADC_UBus_Gain (11.111f)

typedef struct
{
    float Current_A;
    float Current_B;
    float Voltage_A;
    float Voltage_B;
    float Voltage_C;
    float Voltage_Bus;

}ADC_data;

typedef struct
{
    int Current_A;
    int Current_B;
    int Voltage_A;
    int Voltage_B;
    int Voltage_C;
    int Voltage_Bus;
}ADC_Raw_data;

typedef struct
{
    int angle_raw;
    int angle_diff;
    float omega_mec;
}ADC_Angle_data;


void ADC_Manager_Init(void);
void ADC_Angle_Extract();

#endif
