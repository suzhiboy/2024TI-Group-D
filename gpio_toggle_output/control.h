#ifndef __CONTROL_H_
#define __CONTROL_H_

void Control_System_Init(void);
void System_Control_Loop(void);
static void Direction_Outer_Loop_Control(void);
static void Speed_Inner_Loop_Control(void);

#endif