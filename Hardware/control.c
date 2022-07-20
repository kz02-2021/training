#include "control.h"
/************************
直立环控制参数:KP:410,KD:0.133

480	0.056	270 . 1000
************************/
float Vertical_Kp=450;
float Vertical_Kd=0.056;

//float test = 0.0f;
//float test_last = 0.0f;
//float test_d = 0.0f;
//float test_sum = 0.0f;
//float test_ki = 3.7f;
//float test_limit = 1000.0f;
//float angle_limit = 0.01f;
//int v_sum = 0;


/*************************
速度环控制参数:KP:500,KI:600/200
*************************/
float Velocity_Kp=270;
float Velocity_Ki=270/200;
const float max_encoder = 1000;

/*************************
转向环控制参数:KP:-0.05,KP2:0.3
**************************/
float Turn_Kp=-0.07;/*符号为负*/
float Turn_Kp2=0.3;

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
巡线部分参数
***************************/
float Follow_Kp=0;
float Follow_Kd=0;

float Follow_Kp1=0;
float Follow_Kd1=0;
/*************************
PWM输入函数
**************************/
void Load(int moto1,int moto2)//moto1=-200：反转200个脉冲
{
	//1.研究正负号，对应正反转
	if(moto1>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_RESET) ;
								__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,my_abs(moto1 )+250);}//正转
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_SET) ;
							__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,my_abs(moto1 )+400);  }//反转
	//2.研究PWM值
	
	
	if(moto2>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_RESET) ;
							__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,my_abs(moto2 )+250);}
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_SET) ;
							__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,my_abs(moto2 )+400);}	
	
}

/*********************
直立环PD控制器：Kp*Ek+Kd*Ek_D

入口：期望角度、真实角度、真实角速度
出口：直立环输出
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;
//	test = Angle-Med;
//	test_d = test-test_last;
//	if(test > 0.24 || test < -0.24){
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);
//		PWM_out=Vertical_Kp*test-Vertical_Kd*gyro_Y + test_ki*test_sum;//【1】 测试代码
//	}
//	else PWM_out = 0;
//	
//	test_sum = test_sum + test;
//	if (test_sum > test_limit ){
//		test_sum = test_limit;}
//	else if(test_sum < -test_limit){
//		test_sum = -test_limit;
//	}
//		
//	test_last = test;
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
	Encoder_S+=EnC_Err_Lowout;//【4】
	//4.积分限幅
	Encoder_S=Encoder_S>max_encoder?max_encoder:(Encoder_S<(-max_encoder)?(-max_encoder):Encoder_S);
	//5.速度环控制输出计算
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//【5】
//	v_sum = Encoder_S;
	return PWM_out;
}



/*********************
转向环：系数*Z轴角速度
*********************/
int Turn(int gyro_Z,int speed)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z+Turn_Kp2*speed ;
	return PWM_out;
}
/*********************
巡线部分：PID
*********************/
int Follow(short theta,short rho)
{
			static int ErrorTheta,ErrorRho,PWM_out,ErrorTheta_last=0,ErrorRho_last=0;
			ErrorTheta=theta;
			ErrorRho=rho;
			PWM_out= Follow_Kp*ERROR+Follow_Kd*(ErrorTheta-ErrorTheta_last)+Follow_Kp1*(int)rho+Follow_Kd1*((int)ErrorRho-(int)ErrorRho_last);
			ErrorRho_last=ErrorRho;
			ErrorTheta_last=ErrorTheta ;
			return PWM_out;
}




