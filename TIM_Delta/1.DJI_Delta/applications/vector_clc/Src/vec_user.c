//#include "vec_user.h"


////Vector3 o_v_Warr[3];//轮子初始位置向量
////Vector3 Warr[3];//轮子转动方向向量
////Vector3 v_tool_v = {0,0,0};
////Vector3 *vtv_ptr = &v_tool_v;
////Vector3 sub_varr[3];











//extern float v_omega;
//float SO_R = 380;

////Vector3 o_v_Warr[3];//轮子初始位置向量
////Vector3 o_v_Warr[3]={
//// {SO_R*cos(PI/2+v_omega),SO_R*sin(PI/2+v_omega),0},
//// {SO_R*cos(-PI/6+v_omega),SO_R*sin(-PI/6+v_omega),0},
//// {SO_R*cos(-5*PI/6+v_omega),SO_R*sin(-5*PI/6+v_omega),0}
////};//轮子初始位置向量
////o_v_Warr[0]={O_R*cos(PI/2+v_omega),O_R*sin(PI/2+v_omega),0};
////o_v_Warr[1]={O_R*cos(-PI/6+v_omega),O_R*sin(-PI/6+v_omega),0};
////o_v_Warr[2]={O_R*cos(-5*PI/6+v_omega),O_R*sin(-5*PI/6+v_omega),0};
//Vector3 Warr[3];//轮子转动方向向量
//Vector3 sub_varr[3];
//float W_speed[3];

//Vector3 v_tool_v;
//Vector3 *vtv_ptr = &v_tool_v;

//void set_v_tool_v(Vector3 *v_tool_v,float x,float y,float z){
//	v_tool_v->x=x;
//	v_tool_v->y=y;
//	v_tool_v->z=z;
//}
////差向量
//void sub_clac(Vector3 *subarr){
//  subarr[0] = Vector_Sub(v_tool_v , o_v_Warr[0]);
//  subarr[1] = Vector_Sub(v_tool_v , o_v_Warr[1]);
//  subarr[2] = Vector_Sub(v_tool_v , o_v_Warr[2]);
//}
////轮子速度计算
//void speed_cla(float *speed){
//  float L1 = Vector_Dot(sub_varr[0],Warr[0]);
//  float L2 = Vector_Dot(sub_varr[1],Warr[1]);
//  float L3 = Vector_Dot(sub_varr[2],Warr[2]);
// 
//  speed[0] = L1 * Vector_Length(Warr[0]);
//  speed[1] = L2 * Vector_Length(Warr[1]);
//  speed[2] = L3 * Vector_Length(Warr[2]);
//}
 



