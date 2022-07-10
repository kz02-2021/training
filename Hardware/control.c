#include "control.h"
/************************
ֱ�������Ʋ���
************************/
const float Vertical_Kp=300;
const float Vertical_Kd=0.1;

float Vertical_Kp_=800;
float Vertical_Kd_=0.1;

float min_angle = 1;

/*************************
�ٶȻ����Ʋ���
*************************/

const float Velocity_Kp=1.5;		//1.245
float Velocity_Ki=0.01;
int max_es = 10000000;


/*************************
ת�򻷿��Ʋ���
**************************/
float Turn_Kp=0.13;
float Turn_Ki=-10;

/*************************
����ֵ����
**************************/
int my_abs(int i)
{
	if(i<=0)i=-i;
	else i=i;
	return i;
}


/*************************
PWM���뺯��
**************************/
void Load(int moto1,int moto2)//moto1=-200����ת200������
{
	//1.�о������ţ���Ӧ����ת
	if(moto1>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_RESET) ;}//��ת
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_12,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_13,GPIO_PIN_SET) ;}//��ת
	//2.�о�PWMֵ
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,my_abs(moto1 ));
	
	if(moto2>0)	{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_SET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_RESET) ;}
	else 				{HAL_GPIO_WritePin (GPIOB,GPIO_PIN_14,GPIO_PIN_RESET) ;HAL_GPIO_WritePin (GPIOB,GPIO_PIN_15,GPIO_PIN_SET) ;}	
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,my_abs(moto2 ));
}

/*********************
ֱ����PD��������Kp*Ek+Kd*Ek_D

��ڣ������Ƕȡ���ʵ�Ƕȡ���ʵ���ٶ�
���ڣ�ֱ�������
*********************/
int Vertical(float Med,float Angle,float gyro_Y)
{
	int PWM_out;	
	PWM_out=Vertical_Kp*(Angle-Med)+Vertical_Kd*(gyro_Y-0);//��1��
	if (Angle-Med > min_angle || Angle-Med < -min_angle){
		if (Angle > 0) 	PWM_out = Vertical_Kp_*(Angle-Med)+Vertical_Kd_*(gyro_Y-0)-(Vertical_Kp_-Vertical_Kp)*(min_angle-Med);
		else			PWM_out = Vertical_Kp_*(Angle-Med)+Vertical_Kd_*(gyro_Y-0)-(Vertical_Kp_-Vertical_Kp)*(-min_angle-Med);
	}
	return PWM_out;
}

/*********************
�ٶȻ�PI��Kp*Ek+Ki*Ek_S
*********************/
int Velocity(int target_speed,int encoder_left,int encoder_right)
{
	static int PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//��2��
	float a=0.8;//��3��
	
	//1.�����ٶ�ƫ��
	Encoder_Err=(encoder_left+encoder_right)-target_speed ;//��ȥ���
	//2.���ٶ�ƫ����е�ͨ�˲�
	EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//ʹ�ò��θ���ƽ�����˳���Ƶ���ţ���ֹ�ٶ�ͻ�䡣
	EnC_Err_Lowout_last=EnC_Err_Lowout;//��ֹ�ٶȹ����Ӱ��ֱ����������������
	//3.���ٶ�ƫ����֣����ֳ�λ��
	//4.�����޷�
	Encoder_S+=EnC_Err_Lowout;
	Encoder_S=Encoder_S>max_es?max_es:(Encoder_S<(-max_es)?(-max_es):Encoder_S);
	//5.�ٶȻ������������
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//��5��
	return PWM_out;
}



/*********************
ת�򻷣�ϵ��*Z����ٶ�
*********************/
int Turn(int speed, int encoder_loss)
{
	static int Encoder_S_loss;
	int PWM_out;
	Encoder_S_loss += encoder_loss;
	PWM_out=Turn_Kp*speed + Encoder_S_loss*Turn_Ki;
	return PWM_out;
}

