#include "Communication.h"
#include "Task.h"
#include "usart.h"
#include "Control_Loop.h"
#include "arm_math.h"
#include "string.h"

#define BUFFER_SIZE 100

uint8_t UART_BUFFER[BUFFER_SIZE];
uint8_t uart_tail[] = {0x00, 0x00, 0x80, 0x7f};
uint64_t count;


void Communication_Init(void){

    count = 0;

}


#define N_CH 6
void Comnunication_Loop(void){

    static uint8_t data_pack[sizeof(float)*10+4];
    memcpy(data_pack, Control_Loop_Get_test(), sizeof(float)*N_CH);
    memcpy(data_pack + sizeof(float)*N_CH, uart_tail, 4);


#ifndef ENCODER_CALIBRATION

    HAL_UART_Transmit_DMA(&huart3, data_pack, sizeof(float)*N_CH + 4);

#else
    HAL_UART_Transmit(&huart3, (uint8_t*)tmp_dat, sizeof(float)*3, 0xFFFF);
    HAL_UART_Transmit(&huart3, tail, 4, 0xFFFF);
#endif



    count = (count+1)%10000;
        
}
