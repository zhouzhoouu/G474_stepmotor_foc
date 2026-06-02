#include "Control_Loop.h"
#include "HRTIM_Manager.h"
#include "arm_math.h"

static float adc_read_test[10];
static float theta = 0;
static float bais_theta_ele = 0.f;

static inline float mt6816_electrical_angle_normalized(uint32_t raw) {
    uint32_t mod = ((uint32_t)raw * 50) % 2097152;
    float angle = (float)mod * (360.f / 2097152.0f);   // [0, 360)
    if (angle >= 180.0f) angle -= 360.0f;           // 映射到 [-180,180)
    return -angle;
}

void Control_Manager_Loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data){

    theta += Ts*360.f*0.5f;
    if(theta > 360.f)
        theta -= 360;

    float ia = (float)ADC_raw_read->Current_A * ADC_Basic_Gain * ADC_IA_Gain;
    float ib = (float)ADC_raw_read->Current_B * ADC_Basic_Gain * ADC_IB_Gain;
    float ubus = (float)ADC_raw_read->Voltage_Bus * ADC_Basic_Gain * ADC_UBus_Gain;
    float angle_mec = (float)angle_data->angle_raw * (360.f/2097152.f);

    float ua = (float)ADC_raw_read->Voltage_A * ADC_Basic_Gain * ADC_UBus_Gain;
    float ub = (float)ADC_raw_read->Voltage_B * ADC_Basic_Gain * ADC_UBus_Gain;
    float uc = (float)ADC_raw_read->Voltage_C * ADC_Basic_Gain * ADC_UBus_Gain;
    float angle_ele = mt6816_electrical_angle_normalized(angle_data->angle_raw);


    float s,c;
    arm_sin_cos_f32(angle_ele + bais_theta_ele, &s, &c);

    float id = ia * c + ib * s;
    float iq = -ia * s + ib * c;


    float ref_D_d = 0.05f;
    float ref_D_q = 0.f;

    float ref_D_a = ref_D_d * c - ref_D_q * s;
    float ref_D_b = ref_D_d * s + ref_D_q * c;

//    HRTIM_Manager_Ctrl_Set_A(ref_D_b);
//    HRTIM_Manager_Ctrl_Set_B(ref_D_a);
    float sin_f, cos_f;
    arm_sin_cos_f32(theta, &sin_f, &cos_f);
//    HRTIM_Manager_AB_Stop();
    HRTIM_Manager_Ctrl_Set_A(0.25f*cos_f);
    HRTIM_Manager_Ctrl_Set_B(0.25f*sin_f);
    HRTIM_Manager_Ctrl_Set_C(0.f);


    adc_read_test[0] = angle_ele/180.f;
    adc_read_test[1] = id;
    adc_read_test[2] = iq;
    adc_read_test[3] = theta/180.f - 1.f;

}

float* Control_Loop_Get_test(void){
    return adc_read_test;
}
