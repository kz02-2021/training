#include "control.h"
/************************
ֱ�������Ʋ���
************************/
const float Vertical_Kp=200;
const float Vertical_Kd=0.16;

float Vertical_Kp_=800;
float Vertical_Kd_=Vertical_Kd;

float min_angle = 1;

/*************************
�ٶȻ����Ʋ���
*************************/

const float Velocity_Kp=1.4;		//1.245
float Velocity_Ki=Velocity_Kp/200;



/*************************
ת�򻷿��Ʋ���
**************************/
float Turn_Kp=-0.13;
float Turn_Kp2=0;

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
		else						PWM_out = Vertical_Kp_*(Angle-Med)+Vertical_Kd_*(gyro_Y-0)-(Vertical_Kp_-Vertical_Kp)*(-min_angle-Med);
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
	Encoder_S+=EnC_Err_Lowout;//��4��
	//4.�����޷�
	Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
	//5.�ٶȻ������������
	PWM_out=Velocity_Kp*EnC_Err_Lowout+Velocity_Ki*Encoder_S;//��5��
	return PWM_out;
}



/*********************
ת�򻷣�ϵ��*Z����ٶ�
*********************/
int Turn(int gyro_Z,int speed)
{
	int PWM_out;
	
	PWM_out=Turn_Kp*gyro_Z+Turn_Kp2;
	return PWM_out;
}
