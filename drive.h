#ifndef _DRIVE_H_
#define _DRIVE_H_


#define MAX_PWM 255        //PWMの最大値
#define DRIVEN_MOTOR_NUM 4 //駆動のモーターの数

extern double pid_x[3];
extern double pid_y[3];
extern double pid_r[3];
extern double drive_gain[DRIVEN_MOTOR_NUM][3];

struct Point{
public:
	double x;
	double y;
	double r;
	Point(double x_,double y_,double r_){
		x = x_;
		y = y_;
		r = r_;
	}
};
class Motor{
public:
	int motor[30];
	int speed[30];
	int motor_num;
	int smooth_slow;
	int smooth_normal;
	int rising_threshold;
	int small_pwm;
	int stop_flag = 0;
	void rising();
	void update();
	Motor(int motor_num_,int smooth_slow_,int smooth_normal_,int rising_threshold_,int small_pwm_){
		this->motor_num = motor_num_;
		this->smooth_slow = smooth_slow_;
		this->smooth_normal = smooth_normal_;
		this->rising_threshold = rising_threshold_;
		this->small_pwm = small_pwm_;
	}
};
class Drive{
public:
	Motor driven_motor = Motor(DRIVEN_MOTOR_NUM,0,0,0,0);
	int motor[DRIVEN_MOTOR_NUM];
	Point to = Point(0,0,0);
	Point now = Point(0,0,0);
	Point diff = Point(0,0,0);
	Point sum = Point(0,0,0);
	void move(Point p);
	void relativeMove(Point p);
	double absoluteMove();
	void searchPosition(double enc_x,double enc_y);
	void update();
	Drive(Motor motor_){
		this->driven_motor = motor_;
	}
};
double map(int x,int from_min,int from_max,int to_min,int to_max);

#endif 