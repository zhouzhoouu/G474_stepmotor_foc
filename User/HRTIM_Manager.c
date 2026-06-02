#include "HRTIM_Manager.h"
#include "hrtim.h"

void HRTIM_Manager_Init(void){
    LL_HRTIM_TIM_CounterEnable(HRTIM,\
        LL_HRTIM_TIMER_MASTER|\
        LL_HRTIM_TIMER_A|\
        LL_HRTIM_TIMER_B|\
        LL_HRTIM_TIMER_C|\
        LL_HRTIM_TIMER_D|\
        LL_HRTIM_TIMER_E|\
        LL_HRTIM_TIMER_F);
    
}

void HRTIM_Manager_Start(void){

    LL_GPIO_SetOutputPin(EN_GPIO_Port,EN_Pin);
    LL_HRTIM_EnableOutput(HRTIM,\
        LL_HRTIM_OUTPUT_TA1|LL_HRTIM_OUTPUT_TA2 |\
        LL_HRTIM_OUTPUT_TB1|LL_HRTIM_OUTPUT_TB2|\
        LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2|\
        LL_HRTIM_OUTPUT_TD1|LL_HRTIM_OUTPUT_TD2|\
        LL_HRTIM_OUTPUT_TE1|LL_HRTIM_OUTPUT_TE2|\
        LL_HRTIM_OUTPUT_TF1|LL_HRTIM_OUTPUT_TF2);

}

void HRTIM_Manager_AB_Stop(void){
    LL_HRTIM_DisableOutput(HRTIM,\
        LL_HRTIM_OUTPUT_TB1|LL_HRTIM_OUTPUT_TB2|\
        LL_HRTIM_OUTPUT_TE1|LL_HRTIM_OUTPUT_TE2);
};

void HRTIM_Manager_Stop(void){
    LL_GPIO_ResetOutputPin(EN_GPIO_Port,EN_Pin);
    LL_HRTIM_DisableOutput(HRTIM,\
        LL_HRTIM_OUTPUT_TA1|LL_HRTIM_OUTPUT_TA2 |\
        LL_HRTIM_OUTPUT_TB1|LL_HRTIM_OUTPUT_TB2|\
        LL_HRTIM_OUTPUT_TC1|LL_HRTIM_OUTPUT_TC2|\
        LL_HRTIM_OUTPUT_TD1|LL_HRTIM_OUTPUT_TD2|\
        LL_HRTIM_OUTPUT_TE1|LL_HRTIM_OUTPUT_TE2|\
        LL_HRTIM_OUTPUT_TF1|LL_HRTIM_OUTPUT_TF2);
}

void inline HRTIM_Manager_Ctrl_Set(float rate){



}


void HRTIM_Manager_Ctrl_Set_A(float rate){

    if(rate>MAXRATE){
        rate = MAXRATE;
    }
    if(rate<-MAXRATE){
        rate = -MAXRATE;
    }

    int32_t cmpv = (int32_t)(D_FULL * rate);
    cmpv /= 4;


    LL_HRTIM_TIM_SetCompare1(HRTIM,LL_HRTIM_TIMER_E,D_QUAD+cmpv);
    LL_HRTIM_TIM_SetCompare2(HRTIM,LL_HRTIM_TIMER_E,D_Half+D_QUAD-cmpv);

}

void HRTIM_Manager_Ctrl_Set_B(float rate){

    if(rate>MAXRATE){
        rate = MAXRATE;
    }
    if(rate<-MAXRATE){
        rate = -MAXRATE;
    }

    int32_t cmpv = (int32_t)(D_FULL * rate);
    cmpv /= 4;

    LL_HRTIM_TIM_SetCompare1(HRTIM,LL_HRTIM_TIMER_B,D_QUAD+cmpv);
    LL_HRTIM_TIM_SetCompare2(HRTIM,LL_HRTIM_TIMER_B,D_Half+D_QUAD-cmpv);

}


void HRTIM_Manager_Ctrl_Set_C(float rate){

    if(rate>MAXRATE){
        rate = MAXRATE;
    }
    if(rate<-MAXRATE){
        rate = -MAXRATE;
    }

    int32_t cmpv = (int32_t)(D_FULL * rate);
    cmpv /= 4;

    LL_HRTIM_TIM_SetCompare1(HRTIM,LL_HRTIM_TIMER_A,D_QUAD-cmpv);
    LL_HRTIM_TIM_SetCompare2(HRTIM,LL_HRTIM_TIMER_A,D_Half+D_QUAD+cmpv);

}
