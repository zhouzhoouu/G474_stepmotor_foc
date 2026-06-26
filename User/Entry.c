#include "Entry.h"
#include "ADC_Manager.h"
#include "HRTIM_Manager.h"
#include "Task.h"

_Noreturn void Entry(void){

    HAL_Delay(100);
    ADC_Manager_Init();
    HRTIM_Manager_Init();

    HRTIM_Manager_Start();

    Task_Init();

//    LL_GPIO_SetOutputPin(LD0_GPIO_Port,LD0_Pin);

    while (1){
        Task_Process();
    }

}


float AHRS_invSqrt(float num)
{
    float halfnum = 0.5f * num;
    float y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}