#ifndef G474_EDC_TMPLATE_CONTROL_LOOP_H
#define G474_EDC_TMPLATE_CONTROL_LOOP_H

#include "ADC_Manager.h"
#include "arm_math.h"

#define SQ_2 (1.41421356237f)
#define ID_START_RADPS (1500.f)
#define abs (x) ((x)>0?(x):-(x))
//R_s = 4.17ohm
#define R_s (4.17f)

//Lq = 7mH
#define L_q (0.001f*6)

//Ld = 11mH
#define L_d (0.001f*6)

//psi_f = 13mWb
#define psi_f (0.001f*9.f)

#define wc (100.f*2*PI)

#define MAX_DUTY (0.95f)


void Control_Manager_Loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data);
void Control_Manager_Init();

float* Control_Loop_Get_test(void);
volatile uint64_t Get_Ctrl_Tik(void);

#endif
