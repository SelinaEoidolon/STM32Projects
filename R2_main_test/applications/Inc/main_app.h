#ifndef MAIN_APP_H
#define MAIN_APP_H
//#include "planner_global.h"

extern int mi200 ;
extern int mi250 ;
extern int mi270 ;
extern uint8_t mi_high_add_flag;
void user_init(void);
void local_velocity_mode_run_with_angle_displacement(void);
void world_velocity_mode_run(void);
void world_displacement_mode_run(void);
void pure_local_velocity_mode_run(void);
void cybergear_control(void);
void msg_control(void);
void delta_control(void);
void delta_enable(void);
void control_loop(void);
void Shot_Task_Handler(void);

#endif
