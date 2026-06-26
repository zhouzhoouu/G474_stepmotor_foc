#include "Communication.h"
#include "Task.h"
#include "usart.h"
#include "Control_Loop.h"
#include "arm_math.h"
#include "string.h"

#define BUFFER_SIZE 100
#define SELF_ID 1

uint8_t UART_BUFFER[BUFFER_SIZE];
uint8_t uart_tail[] = {0x00, 0x00, 0x80, 0x7f};
uint64_t count;


void Communication_Init(void){

    __HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart3,UART_BUFFER,BUFFER_SIZE);
    count = 0;

}


#define N_CH 6
void Comnunication_Loop(void){

    static uint8_t data_pack[sizeof(float)*10+4];
    memcpy(data_pack, Control_Loop_Get_test(), sizeof(float)*N_CH);


#ifndef ENCODER_CALIBRATION
    memcpy(data_pack + sizeof(float)*N_CH, uart_tail, 4);
    HAL_UART_Transmit_DMA(&huart3, data_pack, sizeof(float)*N_CH + 4);

#else
    memcpy(data_pack + sizeof(float)*2, uart_tail, 4);
    HAL_UART_Transmit_DMA(&huart3, data_pack, sizeof(float)*2 + 4);
#endif



    count = (count+1)%10000;
        
}

static void data_slice(const uint8_t *buf, float *out) {
    int i = 0;
    int p_point = 10 + 1;
    int ati = 0;
    int sig = 1;
    if(buf[0]=='-')
    {
        sig = -1;
        i++;
    }
    while (buf[i]!='\n' && i < 10)
    {
        if(buf[i]!='.')
            ati = ati*10 + (buf[i]-'0');
        else
            p_point = i;
        i++;
    }
    float at = (float)(ati*sig);
    while (++p_point<i)
        at*=.1f;
    *out = at;
}

static void Comnunication_Process(void){
    HAL_UART_AbortReceive(&huart3);


    if(UART_BUFFER[0]=='M' && UART_BUFFER[1]=='0'+SELF_ID && UART_BUFFER[3]==':'){

        if(UART_BUFFER[2] == 'P')
        {
            float ang = 0.f;
            data_slice(UART_BUFFER + 4, &ang);
            Control_Loop_Set_Pos(ang);
        } else if(UART_BUFFER[2] == 'S')
        {
            float spd = 0.f;
            data_slice(UART_BUFFER + 4, &spd);
            Control_Loop_Set_Speed(spd);
        } else if(UART_BUFFER[2] == 'C')
        {
            float cur = 0.f;
            data_slice(UART_BUFFER + 4, &cur);
            Control_Loop_Set_Cur(cur);

        }

    }


    HAL_UART_Receive_DMA(&huart3,UART_BUFFER,BUFFER_SIZE);
}

inline void Comnunication_IDLE_Callback(void){
    Task_Insert(Comnunication_Process);
}


