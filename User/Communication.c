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



void Comnunication_Loop(void){

    float tmp_dat[10];
    memcpy(tmp_dat, Control_Loop_Get_test(), sizeof(float)*10);

    HAL_UART_Transmit(&huart3, (uint8_t*)tmp_dat, sizeof(float)*6, 0xFFFF);
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    HAL_UART_Transmit(&huart3, tail, 4, 0xFFFF);

    count = (count+1)%10000;
        
}
