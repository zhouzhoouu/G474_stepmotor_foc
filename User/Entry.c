#include "Entry.h"
#include "ADC_Manager.h"
#include "HRTIM_Manager.h"
#include "Task.h"

_Noreturn void Entry(void){

    ADC_Manager_Init();
    HRTIM_Manager_Init();

    HRTIM_Manager_Start();

    Task_Init();

//    LL_GPIO_SetOutputPin(LD0_GPIO_Port,LD0_Pin);

    while (1){
        Task_Process();
    }

}
