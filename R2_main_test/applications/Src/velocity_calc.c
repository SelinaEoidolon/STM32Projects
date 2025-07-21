#include "include.h"
#include "velocity_calc.h"

#define RADIUS 1
#define MAX_VEL 8500

float theta;

float calculated_velocity[3];

float calc_buffer[3];               //缓存
float calc_buffer_2nd[3];

float rotate_matrix[3][3]=
{

	{  0 , 0 , 0},
	{  0 , 0 , 0},
	{  0 , 0 , 1}

};                                       //旋转变换矩阵

float inverse_transform_matrix[3][3]=
{

	{  0 , 0 , 0},
	{  0 , 0 , 0},
	{  0 , 0 , 1}

};                                       //逆变矩阵

float solution_matrix[3][3] = {

	{-RADIUS , 1/2*RADIUS , 1/2*RADIUS},
	{0 , 0 , 0},
	{1/RADIUS , 1/RADIUS  , 1/RADIUS }

};                                        //运动学正解矩阵

float inverse_solution_matrix[3][3] = {

	{-1 , 0 , -RADIUS},
	{0.5 , 0 , -RADIUS},
	{0.5 , 0 , -RADIUS}

};                                       //运动学逆解算矩阵


void inverse_solution_matrix_init(void)
{
	inverse_solution_matrix[1][1]=sqrt(3)/2;
	inverse_solution_matrix[2][1]=-sqrt(3)/2;
}

void solution_matrix_init(void)
{
	solution_matrix[1][1]=(sqrt(3)/2)*RADIUS;
	solution_matrix[1][2]=(-sqrt(3)/2)*RADIUS;
}

void matrix_multiply(float matrix[3][3],float input_vector[3],float output_vector[3])
{
	float temp[3]={0};
	for (uint8_t i = 0;i< 3; ++i)
	{
			for (uint8_t k = 0; k < 3; ++k)
			{
				temp[i] += matrix[i][k] * input_vector[k];
			}
			output_vector[i]=temp[i];
	}

}

void rotate_matrix_calc(float theta)
{
	rotate_matrix[0][0] = cos(theta);
	rotate_matrix[0][1] = -sin(theta);
	rotate_matrix[1][0] = sin(theta);
	rotate_matrix[1][1] = cos(theta);
//	
//	inverse_transform_matrix[0][0] = transform_matrix[0][0];
//	inverse_transform_matrix[0][1] = -transform_matrix[0][1];
//	inverse_transform_matrix[1][0] = -transform_matrix[1][0];
//	inverse_transform_matrix[1][1] = transform_matrix[1][1];
}

void get_theta(float omega,float dt)
{
	
	theta+=omega * dt * 2 * PI ;
	
	while(theta>2 * PI)
	{
		theta-=2 * PI;
	}

}

float ftabs_maxfff(float num[3])
{
	float max = fabsf(num[0]);
    float b = fabsf(num[1]);
    float c = fabsf(num[2]);
    if (b > max) max = b;
    if (c > max) max = c;
    return max;
}

void vel_control(float exp_vel[3])//更安全的底盘控制
{
	float max_abs_vel=0;
	matrix_multiply(inverse_solution_matrix,exp_vel,calculated_velocity);
	max_abs_vel=ftabs_maxfff(calculated_velocity);
	if(max_abs_vel>MAX_VEL)
	{
		float scale=MAX_VEL/max_abs_vel;
		for(uint8_t i=0;i<3;++i)
		{
			exp_vel[i]*=scale;
		}
		matrix_multiply(inverse_solution_matrix,exp_vel,calculated_velocity);
	}
	CAN1_CMD_1(PID_velocity_realize_1(calculated_velocity[0],1)*1.1f,PID_velocity_realize_1(calculated_velocity[1],2),PID_velocity_realize_1(calculated_velocity[2],3),0);
}


