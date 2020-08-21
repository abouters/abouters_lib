#ifndef _DRIVE_H_
#define _DRIVE_H_

#define MOTOR_KUDO 4        //駆動のモーターの個数
#define MOTOR_NUM 4         //駆動以外のモーターの個数
#define SMOOTH_NORMAL 10    //通常時のスムースの値（一回のループで変化するモーターのpwm）
#define SMOOTH_SLOW 1       //立ち上がり時のスムースの値
#define MAX_PWM 255         //pwmの最大値
#define THRESHOLD_RISING 15 //モーターの立ち上がりに必要なpwm
#define SMALL_PWM_VALUE 10      //十分小さなpwmの値（pwmがこれより小さい値の時、スムースの値がSMOOTH_SLOWとなる）
extern double pid_x[3];
extern double pid_y[3];
extern double pid_r[3];
extern double gain[MOTOR_KUDO][3];

struct Point{
public:
	double x;
	double y;
	double r;
	Point(double x1,double y1,double r1){
		x = x1;
		y = y1;
		r = r1;
	}
};
class Motor{
public:
	int motor[20];
	int speed[20];
	int motor_num;
	int smooth_slow[20];
	int smooth_normal[20];
	int rising_threshold[20];
	void setThreshold(int threshold);
	void setThreshold(int threshold,int witch);
	void setSmooth(int slow,int normal);
	void setSmooth(int slow,int normal,int witch);
	void rising();
	void update();
	Motor(int num){
		motor_num = num;
	}
};
class Drive{
public:
	Motor motor = Motor(MOTOR_KUDO);
	Point to;
	Point now;
	Point power;
	Point diff;
	Point sum;
	void move(Point p);
	void relativeMove(Point p);
	double absoluteMove();
	void searchPosition(double enc_x,double enc_y);
	void update();
};
double map(int x,int from_min,int from_max,int to_min,int to_max);
int setRising(int x,int rising,int max_pwm);

#endif 