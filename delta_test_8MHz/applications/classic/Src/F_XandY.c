#include "F_Resolve.h"

float vl_wheel_ave[4] = {0,0,0,0};//电机速度数组
float offset_angle = 0;//角度偏移量
float *of_ang_ptr = &offset_angle;


void set_velocity(velocity *tool_v, float x, float y) {
	tool_v->v_x = x;
	tool_v->v_y = y;
}
void set_vl_omega(float vl_omega, float val) {
	vl_omega = val;
}
void set_vl_r(float vl_r, float val) {
	vl_r = val;
}
void set_offset_angle(float angle, float val) {
	angle = val;
}

void set_vl_wheel(float *w_arr,velocity v,float omega,float r){
	float angle = PI/3;
	w_arr[0] = v.v_x - omega*r;
	w_arr[1] = -v.v_x * cos(angle) - v.v_y * sin(angle) - omega*r;
	w_arr[2] = -v.v_x * cos(angle) + v.v_y * sin(angle) - omega*r;
}





