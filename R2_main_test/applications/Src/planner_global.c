#include "include.h"
#include "planner_global.h"

// ��������� S �ٶȹ滮��
S_CurvePlanner vx_planner;
S_CurvePlanner vy_planner;
S_CurvePlanner w_planner;

// ȫ�ֳ�ʼ������
void Planner_Global_Init(float dt) {
    SPlanner_Init(&vx_planner, 2500.0f, 500.0f, dt);  // j_max = 5, a_max = 2
    SPlanner_Init(&vy_planner, 2500.0f, 500.0f, dt);
    SPlanner_Init(&w_planner, 600.0f, 200.0f, dt);  // ͨ�����ٶ���Ӧ��
}
