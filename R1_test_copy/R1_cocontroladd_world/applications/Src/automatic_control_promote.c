#include "include.h"
#include "automatic_control_promote.h"


float low_pass_filter(float input, float last_output,float alpha)
{
	
	return alpha*input+(1.0f-alpha)*last_output;
}

speed_designer body_acc_design[3]={0};
float body_feedforword[3]={0};
float body_accs[3]={0};

void vel_designer_init(speed_designer*this,float max_acc)
{
	
	for(uint8_t i=0;i<3;++i)
	{
		
		this[i].max_acc=max_acc;
		
	}
	
}

void designer_update(speed_designer* this)
{
	
	float delta_v = this->expected_vel - this->current_vel;
    float max_delta = this->max_acc * DT;

    if (fabsf(delta_v) <= max_delta) {
        this->current_vel = this->expected_vel;
        this->current_acc = 0.0f;
    } else {
        float step = (delta_v > 0.0f) ? max_delta : -max_delta;
        this->current_vel += step;
        this->current_acc = step / DT;
    }
	
}

void feedforword_control(speed_designer*this,float *exp_vel)
{
	for(uint8_t i=0;i<3;++i)
	{
		this[i].expected_vel=exp_vel[i];
		designer_update(&this[i]);
		body_feedforword[i]=this[i].current_acc;
	}
	
	matrix_multiply(inverse_solution_matrix,body_feedforword,body_accs);
	
}

spatium_designer xyw_spatium_designer[3]={0};
spatium_designer delta_designer={0};

void spatium_designer_init(spatium_designer*this,float v_m,float a_m)
{
	
	this->v_max=v_m;
	this->a_max=a_m;
	
	this->current_s=0;
	this->current_v=0;
	this->current_t=0;
	
}

void triple_spatium_designer_init(spatium_designer*this,float v_m,float a_m)
{
	
	for(uint8_t i=0;i<3;++i)
	{
		
		spatium_designer_init(&this[i],v_m,a_m);
		
	}
	
}

void spatium_designer_set_target(spatium_designer*this,float total_spatium)
{
	
	this->total_spatium=total_spatium;
	this->finished=False;
	this->current_s=0;
	this->current_v=0;
	this->current_t=0;
	
	this->t[0]=this->v_max/this->a_max;
	this->spatium_acc=0.5f*this->a_max*this->t[0]*this->t[0];
	this->spatium_const=this->total_spatium-2*this->spatium_acc;
	
	if(this->total_spatium>this->spatium_acc)
	{
		this->t[1]=(this->total_spatium-2.0f*this->spatium_acc)/this->v_max;
	}else{/*v_m不可到达*/
		this->t[0] = sqrtf(this->total_spatium / this->a_max);
		this->v_max = this->a_max * this->t[0];
		this->t[1] = 0;
	};
	
}

void spatium_designer_update(spatium_designer*this)
{
	if(this->total_spatium>0)
	{
		if(this->current_t<this->t[0])
		{
			this->current_s+=this->current_v*DT+0.5f*this->a_max*DT*DT;
			this->current_v+=this->a_max*DT;
		}//加速段
		else if(this->current_t<this->t[0]+this->t[1])
		{
			this->current_v=this->v_max;
			this->current_s+=this->v_max*DT;
		}//匀速段
		else
		{
			this->current_s+=this->current_v*DT;
			this->current_v-=this->a_max*DT;
			if (this->current_v < 0) this->current_v = 0;
		}//减速段
		
		if(this->current_s >= this->total_spatium || this->current_t > (2*this->t[0] + this->t[1]))
		{
			this->current_s = this->total_spatium;
			this->current_v = 0;
			this->current_t = 0;
			this->finished=True;
		}
	}
	else
	{
		if(this->current_t<this->t[0])
		{
			this->current_s-=this->current_v*DT+0.5f*this->a_max*DT*DT;
			this->current_v-=this->a_max*DT;
		}//加速段
		else if(this->current_t<this->t[0]+this->t[1])
		{
			this->current_v=this->v_max;
			this->current_s-=this->v_max*DT;
		}//匀速段
		else
		{
			this->current_s-=this->current_v*DT;
			this->current_v+=this->a_max*DT;
			if (this->current_v > 0) this->current_v = 0;
		}//减速段
		
		if(this->current_s >= this->total_spatium || this->current_t > (2*this->t[0] + this->t[1]))
		{
			this->current_s = this->total_spatium;
			this->current_v = 0;
			this->current_t = 0;
			this->finished=True;
		}
	}
	this->current_t+=DT;
	
}



