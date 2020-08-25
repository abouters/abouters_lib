#include "drive.h"
#include "smooth.h"
#include "math.h"

double pid_x[3] = {0,0,0};
double pid_y[3] = {0,0,0};
double pid_r[3] = {0,0,0};


double drive_gain[DRIVEN_MOTOR_NUM][3] = {{0.0,0.0,0.0},
					 {0.0,0.0,0.0},
					 {0.0,0.0,0.0},
					 {0.0,0.0,0.0}};



void Motor::rising(){
	for(int i=0; i<motor_num; i++){
		if(motor[i]>0){
			this->motor[i] = map(speed[i],0,MAX_PWM,rising_threshold,MAX_PWM);
		}else if(motor[i]<0){
			this->motor[i] = map(speed[i],0,-MAX_PWM,-rising_threshold,-MAX_PWM);
		}else{
			this->motor[i] = 0;
		}
	}
}
void Motor::update(){
	rising();
	if(stop_flag){
		for(int i=0; i<motor_num; i++){
			this->motor[i] = 0;
		}
	}else{
		for(int i=0; i<motor_num; i++){
			smoothRising(&(this->motor[i]),speed[i],smooth_normal,smooth_slow,small_pwm);
		}
	}
}
void Drive::move(Point p){
	for(int i=0; i<DRIVEN_MOTOR_NUM; i++){
		this->driven_motor.speed[i] = drive_gain[i][0] * p.x + drive_gain[i][1] * p.y + drive_gain[i][2] * p.r;
	}
}
void Drive::update(){
	this->driven_motor.update();
	for(int i=0; i<DRIVEN_MOTOR_NUM; i++){
		this->motor[i] = driven_motor.motor[i];
	}
}
void Drive::relativeMove(Point p){
	Point tem_p = Point(0,0,0);
	double cosR = cos(now.r);
	double sinR = sin(now.r);
	tem_p.x = p.x*cosR - p.y*sinR;
	tem_p.y = p.x*sinR + p.y*cosR;
	tem_p.r = p.r;
	move(tem_p);
}
double Drive::absoluteMove(){
	Point now_diff = Point(to.x-now.x,to.y-now.y,to.r-now.r);
	int distance;
	if(fabs(now_diff.x)<20){
		sum.x += now_diff.x;
	}else{
		sum.x = 0;
	}
	if(fabs(now_diff.y)<20){
		sum.y += now_diff.y;
	}else{
		sum.y = 0;
	}
	if(fabs(now_diff.r)<0.2){
		sum.r += now_diff.r;
	}else{
		sum.r = 0;
	}
	Point power = Point(0,0,0);
	power.x = pid_x[0]*(now_diff.x) + pid_x[1]*sum.x - pid_x[2]*(diff.x - now_diff.x);
	power.y = pid_y[0]*(now_diff.y) + pid_y[1]*sum.y - pid_y[2]*(diff.y - now_diff.y);
	power.r = pid_r[0]*(now_diff.r) + pid_r[1]*sum.r - pid_r[2]*(diff.r - now_diff.r);
	relativeMove(power);
	diff.x = now_diff.x;
	diff.y = now_diff.y;
	diff.r = now_diff.r;
	distance = sqrt(diff.x*diff.x+diff.y*diff.y);
	return distance;
}
void Drive::searchPosition(double enc_x,double enc_y){
	double cosR = cos(now.r);
	double sinR = sin(now.r);
	now.x += enc_x*cosR - enc_y*sinR;
	now.y += enc_x*sinR + enc_y*cosR;
}
double map(int x,int from_min,int from_max,int to_min,int to_max){
	return ((x - from_min)*(to_max - to_min))/(from_max - from_min) + to_min;
}