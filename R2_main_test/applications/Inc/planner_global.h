#ifndef PLANNER_GLOBAL_H
#define PLANNER_GLOBAL_H

#include "automatic_control_promote.h"

// ����ȫ���ٶȹ滮��
extern S_CurvePlanner vx_planner;
extern S_CurvePlanner vy_planner;
extern S_CurvePlanner w_planner;

// ��ʼ������
void Planner_Global_Init(float dt);

#endif
