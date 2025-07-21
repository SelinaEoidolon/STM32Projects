#ifndef DISPLACEMENT_PID
#define DISPLACEMENT_PID

typedef struct{
	
	float x_error[2];
	float y_error[2];
	float omega_error[2];
	float s_x_error;
	float s_y_error;
	float s_omega_error;

	float kp;
	float kd;
	float ki;
	
	float max_iout;
	float max_out;
	
	float iout[3];
	float pout[3];
	float dout[3];
	
	float out[3];

}displacement_PID;

extern displacement_PID my_displacement_pid;
extern float displace_buffer[3];
extern float motors_rnd[3];

void get_rnd_count_and_diaplacement(void);
void displacement_pid_config(float kp,float kd,float ki,float max_iout,float max_out);
void displacement_control(float x,float y);
void displacement_control_with_omega(float x,float y,float omega);

#endif
