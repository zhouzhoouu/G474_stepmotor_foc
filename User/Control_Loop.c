#include "Control_Loop.h"
#include "HRTIM_Manager.h"
#include "arm_math.h"

static float adc_read_test[10];
static  float theta_fix[100] = {
    4.695723f, 7.279858f, 8.330539f, 9.722947f, 10.829264f, 12.236191f, 12.883788f, 14.165982f, 14.578734f, 15.231578f,
    14.967734f, 15.880703f, 15.726100f, 15.539357f, 15.451748f, 14.272311f, 13.028687f, 11.977505f, 10.840376f, 9.747359f,
    8.898052f, 8.274694f, 7.205293f, 6.917718f, 6.135489f, 6.100544f, 6.009044f, 4.966911f, 3.610823f, 2.238093f,
    0.875548f, -0.940399f, -2.533421f, -4.312339f, -6.338279f, -7.711901f, -9.586229f, -9.987953f, -10.752323f, -10.883035f,
    -11.654126f, -12.140517f, -12.258874f, -12.050196f, -11.986411f, -11.310440f, -11.273141f, -9.935850f, -9.065744f, -8.206124f,
    -6.129610f, -4.211427f, -3.296869f, -1.965777f, -1.008428f, -0.126648f, 1.392996f, 2.056561f, 2.894988f, 3.563537f,
    4.188831f, 4.330483f, 5.115436f, 5.204417f, 5.615689f, 4.755997f, 4.156698f, 3.541487f, 2.854118f, 2.378940f,
    2.380573f, 1.667522f, 1.776208f, 1.527788f, 2.300588f, 2.313559f, 3.175064f, 2.490034f, 2.280532f, 1.410874f,
    0.776904f, -0.307219f, -1.186969f, -2.718498f, -3.490232f, -4.547565f, -5.310493f, -6.013462f, -5.433154f, -5.300272f,
    -5.608562f, -5.499059f, -4.870069f, -4.357305f, -3.272651f, -2.624380f, -1.338318f, -0.082928f, 1.064026f, 2.999463f
};


static inline float mt6816_electrical_angle_normalized(uint32_t raw) {
    uint32_t mod = ((uint32_t)raw * 50) % 2097152;
    float angle = (float)mod * (360.f / 2097152.0f);   // [0, 360)

#ifndef ENCODER_CALIBRATION
    uint32_t n_compensation = ((uint32_t)raw * 100) / 2097152;
    uint32_t n_tail = raw*100 - n_compensation*2097152;
    float f1 = theta_fix[n_compensation];
    float f2 = 0;
    if (n_compensation < 99) {
        f2 = theta_fix[n_compensation + 1];
    } else {
        f2 = theta_fix[0];
    }
    float k = n_tail*(1/2097152.f);
    angle += f1 + k*(f2-f1); // 插值补偿
#endif
    if (angle >= 180.0f) angle -= 360.0f;           // 映射到 [-180,180)
    return angle;
}

static inline void Encoder_calibration_loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data)
{
    static float theta = 0;
    theta += Ts*360.f*0.5f;
    if(theta > 360.f)
        theta -= 360;

    float angle_ele = mt6816_electrical_angle_normalized(angle_data->angle_raw);

    float sin_f, cos_f;
    arm_sin_cos_f32(theta, &sin_f, &cos_f);
    HRTIM_Manager_Ctrl_Set_A(0.25f*cos_f);
    HRTIM_Manager_Ctrl_Set_B(0.25f*sin_f);
    HRTIM_Manager_Ctrl_Set_C(0.f);

    adc_read_test[0] = angle_ele;
    if (theta > 180.f) {
        adc_read_test[1] = theta - 360.f;
    } else {
        adc_read_test[1] = theta;
    }
    adc_read_test[2] = (uint32_t)angle_data->angle_raw;

}

void Control_Manager_Loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data){

#ifdef ENCODER_CALIBRATION
    Encoder_calibration_loop(ADC_raw_read, angle_data);
    return;
#endif

    float ia = (float)ADC_raw_read->Current_A * ADC_Basic_Gain * ADC_IA_Gain;
    float ib = (float)ADC_raw_read->Current_B * ADC_Basic_Gain * ADC_IB_Gain;
    float ubus = (float)ADC_raw_read->Voltage_Bus * ADC_Basic_Gain * ADC_UBus_Gain;
    float angle_mec = (float)angle_data->angle_raw * (360.f/2097152.f);

    float angle_ele = mt6816_electrical_angle_normalized(angle_data->angle_raw);


    float s,c;
    arm_sin_cos_f32(angle_ele, &s, &c);

    float id = ia * c + ib * s;
    float iq = -ia * s + ib * c;


    float ref_D_d = 0.0f;
    float ref_D_q = 0.1f;

    float ref_D_a = ref_D_d * c - ref_D_q * s;
    float ref_D_b = ref_D_d * s + ref_D_q * c;

//     static float theta = 0;
//     theta += Ts*360.f*0.5f;
//     if(theta > 360.f)
//         theta -= 360;
//
//     float sin_f, cos_f;
//     arm_sin_cos_f32(theta, &sin_f, &cos_f);
// //    HRTIM_Manager_AB_Stop();
//     HRTIM_Manager_Ctrl_Set_A(0.25f*cos_f);
//     HRTIM_Manager_Ctrl_Set_B(0.25f*sin_f);

    HRTIM_Manager_Ctrl_Set_A(ref_D_a);
    HRTIM_Manager_Ctrl_Set_B(ref_D_b);
    HRTIM_Manager_Ctrl_Set_C(0.f);

    adc_read_test[0] = angle_ele/180;
    adc_read_test[2] = id;
    adc_read_test[3] = iq;
}


float* Control_Loop_Get_test(void){
    return adc_read_test;
}
