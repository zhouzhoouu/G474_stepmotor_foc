#include "Control_Loop.h"
#include "HRTIM_Manager.h"
#include "dac.h"
#include "arm_math.h"

#define N_test 3

#define LIMTV(x, range) ((x)>(-(range))?((x)<(range)?(x):(range)):(-(range)))

static volatile uint64_t ctrl_tik = 0;
static float adc_read_test[10];

static float mt6835_electrical_angle_normalized(uint32_t raw) {
    uint32_t mod = ((uint32_t)raw * 50) % 2097152;
    float angle = (float)mod * (360.f / 2097152.0f);   // [0, 360)

    if (angle >= 180.0f) angle -= 360.0f;           // 映射到 [-180,180)
    return angle;
}

static void Control_Manager_DAC_Debug(float v){
    v += 1.65f;

    if(v > 3.3f)v = 3.3f;
    if(v < 0) v = 0.f;

    uint32_t tmpv = (uint32_t)((v * (1/3.3f) * 4096));
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, tmpv);
}


void Control_Manager_Init() {
    HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
}

#define acc_tim 3.f
#define max_omega 100.f
static void Encoder_calibration_loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data)
{

    static float time;
    time += Ts;
    float omega = 0;
    if(time/acc_tim<1)
        omega = time/acc_tim*max_omega;
    else
        omega = max_omega;
    float amp = 0.1f+0.2f*omega/max_omega;

    static float theta = 0;
    theta += Ts*360.f*omega;
    if(theta > 360.f)
        theta -= 360;

    float angle_mec = (float)angle_data->angle_raw * (360.f/2097152.f);
    float angle_ele = mt6835_electrical_angle_normalized(angle_data->angle_raw);
//    float angle_ele_from_mec = angle_mec*50.f;
//    while (angle_ele_from_mec > 180.f) angle_ele_from_mec -= 360.f;

    float sin_f, cos_f;
    arm_sin_cos_f32(theta, &sin_f, &cos_f);
    HRTIM_Manager_Ctrl_Set_A(amp*cos_f);
    HRTIM_Manager_Ctrl_Set_B(amp*sin_f);
    Control_Manager_DAC_Debug(angle_ele/180.f);
//    HRTIM_Manager_AB_Stop();
    HRTIM_Manager_Ctrl_Set_C(0.f);

    adc_read_test[0] = angle_ele;
    if (theta > 180.f) {
        adc_read_test[1] = theta - 360.f;
    } else {
        adc_read_test[1] = theta;
    }
    adc_read_test[2] = (float)angle_data->angle_raw;

}

void Control_Manager_Loop(ADC_Raw_data* ADC_raw_read, ADC_Angle_data* angle_data){
    ctrl_tik++;

#ifdef ENCODER_CALIBRATION
    Encoder_calibration_loop(ADC_raw_read, angle_data);
    return;
#endif

    float ia = (float)ADC_raw_read->Current_A * ADC_Basic_Gain * ADC_IA_Gain;
    float ib = (float)ADC_raw_read->Current_B * ADC_Basic_Gain * ADC_IB_Gain;
    float ubus = (float)ADC_raw_read->Voltage_Bus * ADC_Basic_Gain * ADC_UBus_Gain;
    float angle_mec = (float)angle_data->angle_raw * (360.f/2097152.f);

    float angle_ele = mt6835_electrical_angle_normalized(angle_data->angle_raw);
    float omega_ele = (angle_data->omega_mec) * 50.f;

    float s,c;
    arm_sin_cos_f32(angle_ele, &s, &c);

    float id = ia * c + ib * s;
    float iq = -ia * s + ib * c;



    float ang_err = 180.f - angle_mec;

    float iq_ref = ang_err/40.f;
    float id_ref = -.0f;
     float u_ref = 2.4f;

//     static float theta = 0;
//     static float iq_cos = 0.f;
//     static float iq_sin = 0.f;
//     theta += Ts*360.f*10.f;//*85.f*N_test;
//     if(theta > 360.f)
//         theta -= 360;
//
//     float sin_f, cos_f;
//     arm_sin_cos_f32(theta, &sin_f, &cos_f);

    // iq_cos += 0.0005f*(id*cos_f - iq_cos);
    // iq_sin += 0.0005f*(id*sin_f - iq_sin);

    static float ref_d_pi, ref_q_pi, last_err_iq, last_err_id;

    float err_iq = iq_ref - iq;
    ref_q_pi += (err_iq-last_err_iq)*L_q*wc; + err_iq*Ts*R_s*wc;
    last_err_iq = err_iq;

    float err_id = id_ref - id;
    ref_d_pi += (err_id-last_err_id)*L_d*wc; + err_id*Ts*R_s*wc;
    last_err_id = err_id;

//    ref_q_pi = 0.0f;
//    ref_d_pi = 0.0f;

    LIMTV(ref_q_pi, ubus*0.95f);
    LIMTV(ref_d_pi, ubus*0.95f);

    float ref_D_d = ref_d_pi - iq*L_q*omega_ele;
    float ref_D_q = ref_q_pi + omega_ele*(id*L_d+psi_f);

    ref_D_d /= ubus;
    ref_D_q /= ubus;

    float ref_D_a = ref_D_d * c - ref_D_q * s;
    float ref_D_b = ref_D_d * s + ref_D_q * c;

    Control_Manager_DAC_Debug(iq);
//    HRTIM_Manager_AB_Stop();
//     HRTIM_Manager_Ctrl_Set_A(0.15f*cos_f);
//     HRTIM_Manager_Ctrl_Set_B(0.15f*sin_f);

    HRTIM_Manager_Ctrl_Set_A(ref_D_a);
    HRTIM_Manager_Ctrl_Set_B(ref_D_b);
    HRTIM_Manager_Ctrl_Set_C(0.f);

    adc_read_test[0] = angle_ele/180;
    adc_read_test[1] = id;
    adc_read_test[2] = iq;
    adc_read_test[3] = omega_ele;
    adc_read_test[4] = ubus;
}


float* Control_Loop_Get_test(void){
    return adc_read_test;
}

volatile uint64_t Get_Ctrl_Tik(void){
    return ctrl_tik;
}
