#include "Task.h"
#include "tim.h"
#include "ADC_Manager.h"

#include "Communication.h"

#define MAX_TASK_NUM 50

typedef void (*Task_Func)(void);

volatile uint32_t head;
volatile uint32_t tail;
volatile Task_Func RING_TASK_BUFFER[MAX_TASK_NUM];

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM6){
        Task_Insert(ADC_Angle_Extract);
    }
    else if(htim->Instance == TIM7){
        Task_Insert(Comnunication_Loop);
    }
}

void Task_Init(void){

    head = 0;
    tail = 0;

    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

}

inline void Task_Insert(Task_Func task){

    __disable_irq();
    uint32_t h_next = (head+1)%MAX_TASK_NUM;
    if(h_next!=tail){
        RING_TASK_BUFFER[head] = task;
        head = h_next;
    }
    __enable_irq();

}

void Task_Process(void){
    if(tail != head){
        RING_TASK_BUFFER[tail]();
        tail = (tail+1)%MAX_TASK_NUM;
    }

}
