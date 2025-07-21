//#include "Resolve.h"

//double vl_wheel_ave[4] = {0,0,0,0};//电机速度数组
//double offset_angle = 0;//角度偏移量
//double *of_ang_ptr = &offset_angle;


//void set_velocity(velocity *tool_v, double x, double y) {
//	tool_v->v_x = x;
//	tool_v->v_y = y;
//}
//void set_vl_omega(double vl_omega, double val) {
//	vl_omega = val;
//}
//void set_vl_r(double vl_r, double val) {
//	vl_r = val;
//}
//void set_offset_angle(double angle, double val) {
//	angle = val;
//}

//double calc_w2_v(velocity v){
//	double angle = PI / 3;
//	double result2 = 0;
//	if( ABS(v.v_x * cos(angle)) > ABS(v.v_y * sin(angle)) ){
//	   if((v.v_x * cos(angle) + v.v_y * sin(angle)) > 0){
//			 result2 = -(v.v_x * cos(angle) + v.v_y * sin(angle));
//		 }
//		 if((v.v_x * cos(angle) + v.v_y * sin(angle)) < 0){
//			 result2 = -(v.v_x * cos(angle) + v.v_y * sin(angle));
//		 }
//	}
//	if( ABS(v.v_x * cos(angle)) < ABS(v.v_y * sin(angle)) ){
//	   if((v.v_x * cos(angle) + v.v_y * sin(angle)) > 0){
//			 result2 = -(v.v_x * cos(angle) + v.v_y * sin(angle));
//		 }
//		 if((v.v_x * cos(angle) + v.v_y * sin(angle)) < 0){
//			 result2 = -(v.v_x * cos(angle) + v.v_y * sin(angle));
//		 }
//	}
////	if((v.v_x * cos(angle) + v.v_y * sin(angle)) > 0){
////		return -(v.v_x * cos(angle) + v.v_y * sin(angle));
////	}
////	else {
////		return -(v.v_x * cos(angle) + v.v_y * sin(angle));
////	}	
//	return result2;
//}

//double calc_w3_v(velocity v){
//	double angle = PI / 3;
//	double result3= 0;
//	if( ABS(v.v_x * cos(angle)) > ABS(v.v_y * sin(angle)) ){
//		if(v.v_x * cos(angle)>0&&v.v_y * sin(angle)>0){
//			result3 = -(v.v_x * cos(angle) - v.v_y * sin(angle));
//		}
//		if(v.v_x * cos(angle)<0&&v.v_y * sin(angle)<0){
//			result3 = -(v.v_x * cos(angle) - v.v_y * sin(angle));
//		}
//		if(v.v_x * cos(angle)<0&&v.v_y * sin(angle)>0){
//			result3 = -(v.v_x * cos(angle) + v.v_y * sin(angle));
//		}
//		if(v.v_x * cos(angle)>0&&v.v_y * sin(angle)<0){
//			result3 = -(v.v_x * cos(angle) + v.v_y * sin(angle));
//		}
//		
////	   if((v.v_x * cos(angle) - v.v_y * sin(angle)) > 0){
////			 return -(v.v_x * cos(angle) - v.v_y * sin(angle));
////		 }
////		 if((v.v_x * cos(angle) - v.v_y * sin(angle)) < 0){
////			 return -(v.v_x * cos(angle) - v.v_y * sin(angle));
////		 }
//	}
//	if( ABS(v.v_x * cos(angle)) < ABS(v.v_y * sin(angle)) ){
//		if(v.v_x * cos(angle)>0&&v.v_y * sin(angle)>0){
//			result3 = -(v.v_x * cos(angle) - v.v_y * sin(angle));
//		}
//		if(v.v_x * cos(angle)<0&&v.v_y * sin(angle)<0){
//			result3 = -(v.v_x * cos(angle) - v.v_y * sin(angle));
//		}
//		if(v.v_x * cos(angle)<0&&v.v_y * sin(angle)>0){
//			result3 = (v.v_x * cos(angle) + v.v_y * sin(angle));
//		}
//		if(v.v_x * cos(angle)>0&&v.v_y * sin(angle)<0){
//			result3 = (v.v_x * cos(angle) + v.v_y * sin(angle));
//		}
////	   if((v.v_x * cos(angle) - v.v_y * sin(angle)) > 0){
////			 return -(v.v_x * cos(angle) + v.v_y * sin(angle));
////		 }
////		 if((v.v_x * cos(angle) - v.v_y * sin(angle)) < 0){
////			 return -(v.v_x * cos(angle) + v.v_y * sin(angle));
////		 }
//	}
//	return result3;
//}

//void set_vl_wheel(int i, double *warr, velocity v, double omega, double r) {
//	double angle1 = 0;
//	//double angle2 = 0;
//	double rad_offset_angle = offset_angle * PI / 180;//角度单位转换

//	if (i == 3) {
//		if(offset_angle == 0){
//			angle1 = PI / 3;
//			
//			warr[0] = v.v_x + omega * r;
//      warr[1] = calc_w2_v(v) + omega * r;
//		  warr[2] = calc_w3_v(v) + omega * r;
////				}
//		}
//		else if (offset_angle != 0) {
//			angle1 = PI / 3;
////			angle2 = PI / 3;
//			for(int j;j<i;j++){
//				if(j==0){
//			      warr[0] = v.v_x * cos(rad_offset_angle) + v.v_y * sin(rad_offset_angle) + omega * r;
//				}
//				else if(j==1){
//			     	warr[1] = v.v_x * cos(angle1 + rad_offset_angle) + v.v_y * sin(angle1 - rad_offset_angle) + omega * r;

//				}
//				else if(j==2){
//			     	warr[2] = v.v_x * cos(angle1 + rad_offset_angle) + v.v_y * sin(angle1 - rad_offset_angle) + omega * r;

//				}
//			}
//		}
//		
//	}
//	else if (i == 4) {
//		warr[0] = v.v_y + omega * r;
//		warr[1] = v.v_x + omega * r;
//		warr[2] = v.v_y + omega * r;
//		warr[3] = v.v_x + omega * r;
//	}
//}










