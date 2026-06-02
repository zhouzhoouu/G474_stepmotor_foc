#ifndef G474_EDC_DIGPWR_TASK_H
#define G474_EDC_DIGPWR_TASK_H

#include "main.h"

typedef void (*Task_Func)(void);

void Task_Init(void);
void Task_Insert(Task_Func task);
void Task_Process(void);

#endif //G474_EDC_DIGPWR_TASK_H
