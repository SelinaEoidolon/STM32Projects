#ifndef FILTER_AND_OBSERVER_H
#define FILTER_AND_OBSERVER_H

#define DT 0.001           //控制周期为1ms

typedef struct{
	
	int tmep;
	
}state_observer;

typedef enum{
	False=0,
	True
}my_bool;

typedef struct{
	
	float total_spatium;
	
	float v_max;
	float a_max;
	
	float expected_acc;
	float expected_vel;
	
	float spatium_acc;
	float spatium_const;
	
	float current_v;
	float current_s;
	
	float total_t;
	float t[2];
	float current_t;
	
	float v_out;
	my_bool finished;
	
}spatium_designer;

typedef struct{
	
	float expected_vel;
	float current_vel;
	float max_acc;
	float current_acc;
	
}speed_designer;

extern speed_designer body_acc_design[3];
extern float body_feedfowword[3];
extern float body_accs[3];
extern spatium_designer xyw_spatium_designer[3];
extern spatium_designer delta_designer;

float low_pass_filter(float input, float last_output,float alpha);
void vel_designer_init(speed_designer*this,float max_acc);
void triple_spatium_designer_init(spatium_designer*this,float v_m,float a_m);
void designer_update(speed_designer* this);
void feedforword_control(speed_designer*this,float *exp_vel);
void spatium_designer_init(spatium_designer*this,float v_m,float a_m);
void spatium_designer_set_target(spatium_designer*this,float total_spatium);
void spatium_designer_update(spatium_designer*this);

#endif
