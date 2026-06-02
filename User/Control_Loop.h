#ifndef G474_EDC_TMPLATE_CONTROL_LOOP_H
#define G474_EDC_TMPLATE_CONTROL_LOOP_H

#include "ADC_Manager.h"

void Control_Manager_Loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data);

float* Control_Loop_Get_test(void);

#endif
