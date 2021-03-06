#include "control.h"
/************************
直立环控制参数
************************/
const float Vertical_Kp=300;
const float Vertical_Kd=0.1;

float Vertical_Kp_=800;
float Vertical_Kd_=0.1;

float min_angle = 1;

/*************************
速度环控制参数
*************************/

const float Velocity_Kp=1.5;		//1.245
float Velocity_Ki=0.01;
int max_es = 30;


/*************************
转向环控制参数
**************************/
float Turn_Kp=0.13;
float Turn_Ki=-50;

/*************************
绝对值函数
**************************/
int my_abs(int i)
{
	if(i<=0)i=-i;
	else i=i;
	return i;
}


/*************************
PWM输入函数
**************************/
void Load(int moto1,int moto2)//moto1=-200：反转200个脉冲
{
	//1.研究正负号，对应正反转
	if(moto1>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_RESET) ;}//正转
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_SET) ;}//反转
	//2.研究PWM值
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,my_abs(moto1 ));
	
	if(moto2>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_RESET) ;}
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_SET) ;}	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,my_abs(moto2 ));
}

/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);//【1】
	if (Angle-Med > min_angle || Angle-Med < -min_angle){
		if (Angle > 0) 	PWM_out = Vertical_Kp_*(Angle-Med)+Vertical_Kd_*(gyro_Y-0)-(Vertical_Kp_-Vertical_Kp)*(min_angle-Med);
		else			PWM_out = Vertical_Kp_*(Angle-Med)+Vertical_Kd_*(gyro_Y-0)-(Vertical_Kp_-Vertical_Kp)*(-min_angle-Med);
	}
	return PWM_out;
}

/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int target_speed,int encoder_left,int encoder_right)
{
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//【2】
	float a=0.8;//【3】
	
	//1.计算速度偏差
	Encoder_Err=(encoder_left+encoder_right)-target_speed/2000 ;//舍去误差
	//2.对速度偏差进行低通滤波
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
	EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
	//3.对速度偏差积分，积分出位移
	//4.积分限幅
	Encoder_S+=EnC_Err_Lowout;
	Encoder_S=Encoder_S>max_es?max_es:(Encoder_S<(-max_es)?(-max_es):Encoder_S);
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//【5】
	return PWM_out;
}



/*********************
转向环：系数*Z轴角速度
*********************/
int Turn(int speed, int encoder_loss)
{
	static int Encoder_S_loss;
	int PWM_out;
	Encoder_S_loss += encoder_loss;
	Encoder_S_loss=Encoder_S_loss>max_es?max_es:(Encoder_S_loss<(-max_es)?(-max_es):Encoder_S_loss);
	PWM_out=Turn_Kp*speed + Encoder_S_loss*Turn_Ki;

	return PWM_out;
}

