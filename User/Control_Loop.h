#ifndef G474_EDC_TMPLATE_CONTROL_LOOP_H
#define G474_EDC_TMPLATE_CONTROL_LOOP_H

#include "ADC_Manager.h"
#include "arm_math.h"

//R_s = 4.17ohm
#define R_s (4.17f)

//Lq = 7mH
#define L_q (0.001f*7)

//Ld = 11mH
#define L_d (0.001f*11)

//psi_f = 13mWb
#define psi_f (0.001f*14.f)

#define wc (300.f*2*PI)


void Control_Manager_Loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data);

float* Control_Loop_Get_test(void);
volatile uint64_t Get_Ctrl_Tik(void);

#endif
