#ifndef __HRTIM_MANAGER_H
#define __HRTIM_MANAGER_H

#include "main.h"

#define HRTIM HRTIM1
#define MAXRATE 0.97f
#define D_FULL 40000
#define D_Half 20000
#define D_QUAD 10000
#define D_MIN 1024   //10%


#define LIMT(a) (a)>D_MIN?((a)<D_MAX?(a):D_MAX):D_MIN

void HRTIM_Manager_Init(void);
void HRTIM_Manager_Start(void);
void HRTIM_Manager_Stop(void);
void HRTIM_Manager_AB_Stop(void);


void HRTIM_Manager_Ctrl_Set_A(float rate);
void HRTIM_Manager_Ctrl_Set_B(float rate);
void HRTIM_Manager_Ctrl_Set_C(float rate);

#endif