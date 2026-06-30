#include "y_motor.h"


PID A_SpeedPID={0};
PID B_SpeedPID={0};
PID C_SpeedPID={0};
PID D_SpeedPID={0};
int Z_step=0;				//丝杆底部为0
int Y_step=0;				//滑台右侧为0

#define MOTOR_A_SPEED_PID_KP       10.0f
#define MOTOR_A_SPEED_PID_KI       10.0f
#define MOTOR_A_SPEED_PID_KD       0.0f

#define MOTOR_B_SPEED_PID_KP       10.0f
#define MOTOR_B_SPEED_PID_KI       10.0f
#define MOTOR_B_SPEED_PID_KD       0.0f

#define MOTOR_C_SPEED_PID_KP       10.0f
#define MOTOR_C_SPEED_PID_KI       10.0f
#define MOTOR_C_SPEED_PID_KD       0.0f

#define MOTOR_D_SPEED_PID_KP       10.0f
#define MOTOR_D_SPEED_PID_KI       10.0f
#define MOTOR_D_SPEED_PID_KD       0.0f

#define SPEED_PID_INTEGRAL_MAX  1000.0f
#define SPEED_PID_OUTPUT_MAX    3000.0f

static float limit_float(float value, float min_value, float max_value)
{
	if (value > max_value)
		return max_value;
	if (value < min_value)
		return min_value;
	return value;
}

void PID_Reset(PID *sptr)
{
	sptr->LastError = 0.0f;
	sptr->LLastError = 0.0f;
	sptr->IntegralOut = 0.0f;
	sptr->Output = 0.0f;
}

static void PID_InitOne(
	PID *sptr,
	float kp,
	float ki,
	float kd,
	float error_min,
	float error_max)
{
	sptr->Kp = kp;
	sptr->Ki = ki;
	sptr->Kd = kd;
	sptr->IntegralMax = SPEED_PID_INTEGRAL_MAX;
	sptr->OutputMax = SPEED_PID_OUTPUT_MAX;
	sptr->errormin = error_min;
	sptr->errormax = error_max;
	PID_Reset(sptr);
}

/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
返回值：void
********************************************************************************/

void PID_int(void)
{
	PID_InitOne(
		&A_SpeedPID,
		MOTOR_A_SPEED_PID_KP,
		MOTOR_A_SPEED_PID_KI,
		MOTOR_A_SPEED_PID_KD,
		-500.0f,
		350.0f);
	PID_InitOne(
		&B_SpeedPID,
		MOTOR_B_SPEED_PID_KP,
		MOTOR_B_SPEED_PID_KI,
		MOTOR_B_SPEED_PID_KD,
		-350.0f,
		350.0f);
	PID_InitOne(
		&C_SpeedPID,
		MOTOR_C_SPEED_PID_KP,
		MOTOR_C_SPEED_PID_KI,
		MOTOR_C_SPEED_PID_KD,
		-350.0f,
		350.0f);
	PID_InitOne(
		&D_SpeedPID,
		MOTOR_D_SPEED_PID_KP,
		MOTOR_D_SPEED_PID_KI,
		MOTOR_D_SPEED_PID_KD,
		-350.0f,
		350.0f);

/*//中速200
	float SPEED_Kp = 5;
	float SPEED_Ki = 5;
	float SPEED_Kd = 5;
	A_SpeedPID.Kp=5;     //速度环PID参数（D车用，速度环2ms）
	A_SpeedPID.Ki=2;
	A_SpeedPID.Kd=2;
	A_SpeedPID.LastError=0;
	A_SpeedPID.LLastError=0;
	A_SpeedPID.iIncpid=0;
	A_SpeedPID.errormax=300;
	A_SpeedPID.errormin=-500;
	A_SpeedPID.Imax=3500;
	
	B_SpeedPID.Kp=5;       //速度环PID参数（D车用，速度环2ms） 
	B_SpeedPID.Ki=3;
	B_SpeedPID.Kd=2;
	B_SpeedPID.LastError=0;
	B_SpeedPID.LLastError=0;
	B_SpeedPID.iIncpid=0;
	B_SpeedPID.errormax=300;
	B_SpeedPID.errormin=-300;
	B_SpeedPID.Imax=3500;
	
	C_SpeedPID.Kp=SPEED_Kp;       //速度环PID参数（D车用，速度环2ms） 
	C_SpeedPID.Ki=SPEED_Ki;
	C_SpeedPID.Kd=SPEED_Kp;
	C_SpeedPID.LastError=0;
	C_SpeedPID.LLastError=0;
	C_SpeedPID.iIncpid=0;
	C_SpeedPID.errormax=300;
	C_SpeedPID.errormin=-300;
	C_SpeedPID.Imax=3500;
	
	D_SpeedPID.Kp=SPEED_Kp;//速度环PID参数（D车用，速度环2ms） 
	D_SpeedPID.Ki=SPEED_Ki;
	D_SpeedPID.Kd=SPEED_Kp;
	D_SpeedPID.LastError=0;
	D_SpeedPID.LLastError=0;
	D_SpeedPID.iIncpid=0;	
	D_SpeedPID.errormax=300;
	D_SpeedPID.errormin=-300;
	D_SpeedPID.Imax=3500;
*/	
	/*低速，100
	float SPEED_Kp = 5;
	float SPEED_Ki = 3;
	float SPEED_Kd = 2;
	A_SpeedPID.Kp=5;     //速度环PID参数（D车用，速度环2ms）
	A_SpeedPID.Ki=2;
	A_SpeedPID.Kd=2;
	A_SpeedPID.LastError=0;
	A_SpeedPID.LLastError=0;
	A_SpeedPID.iIncpid=0;
	A_SpeedPID.errormax=200;
	A_SpeedPID.errormin=-500;
	A_SpeedPID.Imax=3500;
	
	B_SpeedPID.Kp=5;       //速度环PID参数（D车用，速度环2ms） 
	B_SpeedPID.Ki=3;
	B_SpeedPID.Kd=2;
	B_SpeedPID.LastError=0;
	B_SpeedPID.LLastError=0;
	B_SpeedPID.iIncpid=0;
	B_SpeedPID.errormax=200;
	B_SpeedPID.errormin=-200;
	B_SpeedPID.Imax=3500;
	
	C_SpeedPID.Kp=SPEED_Kp;       //速度环PID参数（D车用，速度环2ms） 
	C_SpeedPID.Ki=SPEED_Ki;
	C_SpeedPID.Kd=SPEED_Kp;
	C_SpeedPID.LastError=0;
	C_SpeedPID.LLastError=0;
	C_SpeedPID.iIncpid=0;
	C_SpeedPID.errormax=200;
	C_SpeedPID.errormin=-200;
	C_SpeedPID.Imax=3500;
	
	D_SpeedPID.Kp=SPEED_Kp;//速度环PID参数（D车用，速度环2ms） 
	D_SpeedPID.Ki=SPEED_Ki;
	D_SpeedPID.Kd=SPEED_Kp;
	D_SpeedPID.LastError=0;
	D_SpeedPID.LLastError=0;
	D_SpeedPID.iIncpid=0;	
	D_SpeedPID.errormax=200;
	D_SpeedPID.errormin=-200;
	D_SpeedPID.Imax=2500;
	*/
}
/************************************************
函数名：IncPIDCalc
功  能：带独立积分限幅和输出限幅的标准增量式PID
参  数：PID状态、目标速度、实际速度、采样周期（秒）
返回值：累计PID输出
************************************************/
float IncPIDCalc(PID *sptr, float Setpoint, float Turepoint, float dt_s)
{
	float error;
	float delta_p;
	float delta_i;
	float delta_d;
	float next_integral;
	float applied_delta_i;
	float next_output;

	if (Setpoint == 0.0f || dt_s <= 0.0f) {
		PID_Reset(sptr);
		return 0.0f;
	}

	error = limit_float(Setpoint - Turepoint, sptr->errormin, sptr->errormax);

	delta_p = sptr->Kp * (error - sptr->LastError);
	delta_i = sptr->Ki * dt_s * error;
	delta_d = (sptr->Kd / dt_s)
		* (error - 2.0f * sptr->LastError + sptr->LLastError);

	next_integral = limit_float(
		sptr->IntegralOut + delta_i,
		-sptr->IntegralMax,
		sptr->IntegralMax);
	applied_delta_i = next_integral - sptr->IntegralOut;

	next_output = sptr->Output + delta_p + applied_delta_i + delta_d;

	/* Do not integrate farther into final-output saturation. */
	if ((next_output > sptr->OutputMax && applied_delta_i > 0.0f)
		|| (next_output < -sptr->OutputMax && applied_delta_i < 0.0f)) {
		next_integral = sptr->IntegralOut;
		next_output = sptr->Output + delta_p + delta_d;
	}

	sptr->IntegralOut = next_integral;
	sptr->Output = limit_float(
		next_output,
		-sptr->OutputMax,
		sptr->OutputMax);
	sptr->LLastError = sptr->LastError;
	sptr->LastError = error;

	return sptr->Output;
}


/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-3000~3000（极限范围-4200~4200） SpeedA,编码器读值
 * @返回值 无
 */
void MOTOR_A_SetSpeed_Close(float speed, float SpeedA, float dt_s)
{ 
	int16_t Atemp;
	float aim_speed_A = speed;
	float OUTPUT_SPEED_A = 0;
	float T_SpeedA =SpeedA;
	
	OUTPUT_SPEED_A = IncPIDCalc(&A_SpeedPID, aim_speed_A, T_SpeedA, dt_s);
	/***
	if (OUTPUT_SPEED_A>=300)
	{
	 OUTPUT_SPEED_A=300;
	}
	*///
	Atemp =OUTPUT_SPEED_A;
	if (Atemp > 3000)
		Atemp = 3000;
	if (Atemp < -3000)
		Atemp = -3000;
	if(aim_speed_A == 0)
	{
		Atemp=0;
	}
	if (Atemp > 0)
	{
		TIM_SetCompare1(TIM1, 4200);
		TIM_SetCompare2(TIM1, (4200 - Atemp));
	}
	else
	{
		TIM_SetCompare2(TIM1, 4200);
		TIM_SetCompare1(TIM1, (4200 + Atemp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-3000~3000（极限范围-4200~4200）
 * @返回值 无
 */
void MOTOR_B_SetSpeed_Close(float speed, float SpeedB, float dt_s)
{
	int16_t Btemp;
	float aim_speed_B = speed;
	float OUTPUT_SPEED_B = 0;
	float T_SpeedB =SpeedB;
	
	OUTPUT_SPEED_B = IncPIDCalc(&B_SpeedPID, aim_speed_B, T_SpeedB, dt_s);
	Btemp = OUTPUT_SPEED_B;

	if (Btemp > 3000)
		Btemp = 3000;
	if (Btemp < -3000)
		Btemp = -3000;

	if(aim_speed_B == 0)
	{
		Btemp=0;
	}
	
	if (Btemp > 0)
	{
		TIM_SetCompare3(TIM1, 4200);
		TIM_SetCompare4(TIM1, (4200 - Btemp));
	}
	else
	{
		TIM_SetCompare4(TIM1, 4200);
		TIM_SetCompare3(TIM1, (4200 + Btemp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_D_SetSpeed_Close(float speed, float SpeedD, float dt_s)
{
	int16_t Dtemp;
	float aim_speed_D = speed;
	float OUTPUT_SPEED_D = 0;
	float T_SpeedD = SpeedD;
	
	OUTPUT_SPEED_D = IncPIDCalc(&D_SpeedPID, aim_speed_D, T_SpeedD, dt_s);
	Dtemp = OUTPUT_SPEED_D;
  
	if (Dtemp > 3000)
		Dtemp = 3000;
	if (Dtemp < -3000)
		Dtemp = -3000;

	if(aim_speed_D == 0)
	{
		Dtemp=0;
	}
	if (Dtemp > 0)
	{
		TIM_SetCompare1(TIM9, 4200);
		TIM_SetCompare2(TIM9, (4200 - Dtemp));
	}
	else
	{
		TIM_SetCompare2(TIM9, 4200);
		TIM_SetCompare1(TIM9, (4200 + Dtemp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_C_SetSpeed_Close(float speed, float SpeedC, float dt_s)
{
	int16_t Ctemp;
	float aim_speed_C = speed;
	float OUTPUT_SPEED_C = 0;
	float T_SpeedC = SpeedC;
	
	
	OUTPUT_SPEED_C = IncPIDCalc(&C_SpeedPID, aim_speed_C, T_SpeedC, dt_s);
	
	Ctemp = OUTPUT_SPEED_C;
	if (Ctemp > 3000)
		Ctemp = 3000;
	if (Ctemp < -3000)
		Ctemp = -3000;
	if(aim_speed_C == 0)
	{
		Ctemp=0;
	}
	if (Ctemp > 0)
	{
		TIM_SetCompare1(TIM12, 4200);
		TIM_SetCompare2(TIM12, (4200 - Ctemp));
	}
	else
	{
		TIM_SetCompare2(TIM12, 4200);
		TIM_SetCompare1(TIM12, (4200 + Ctemp));
	}
}



/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_A_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM1, 4200);
		TIM_SetCompare2(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM1, 4200);
		TIM_SetCompare1(TIM1, (4200 + temp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_B_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare3(TIM1, 4200);
		TIM_SetCompare4(TIM1, (4200 - temp));
	}
	else
	{
		TIM_SetCompare4(TIM1, 4200);
		TIM_SetCompare3(TIM1, (4200 + temp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_D_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM9, 4200);
		TIM_SetCompare2(TIM9, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM9, 4200);
		TIM_SetCompare1(TIM9, (4200 + temp));
	}
}

/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-4200~4200
 * @返回值 无
 */
void MOTOR_C_SetSpeed(int16_t speed)
{
	int16_t temp;

	temp = speed;

	if (temp > 4200)
		temp = 4200;
	if (temp < -4200)
		temp = -4200;

	if (temp > 0)
	{
		TIM_SetCompare1(TIM12, 4200);
		TIM_SetCompare2(TIM12, (4200 - temp));
	}
	else
	{
		TIM_SetCompare2(TIM12, 4200);
		TIM_SetCompare1(TIM12, (4200 + temp));
	}
}

//滑台    左正右负
void Stepper_Motor_Run_1(int32_t num)  
{
	int i;
	int dir;
	
	Y_step+=num;
	
	if(num>0)
	{
	  dir=0;
	}
	if(num<0)
	{
	  dir=1;
	}
	if(dir==0)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_12);
	}
	
	if(dir==1)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_12);		
	}
	for( i=0;i<=abs(num*800);i++)  //电平翻转2次为一个脉冲，所以旋转一周需要3200个脉冲，
	{		
		Delay_us(1000);                   //高低电平持续时间，脉冲频率
		GPIOC->ODR ^= GPIO_Pin_8;  				 //翻转输出电平
	}	
}
//丝杆     上负下正
void Stepper_Motor_Run_2(int32_t num)  
{
	int i;
	int dir;
	
	Z_step+=num;
	
	if(num>0)
	{
	  dir=0;
	}
	if(num<0)
	{
	  dir=1;
	}
	if(dir==0)
	{
		GPIO_SetBits(GPIOC, GPIO_Pin_3);
	}
	
	if(dir==1)
	{
		GPIO_ResetBits(GPIOC, GPIO_Pin_3);			
	}
	for( i=0;i<=abs(num*400);i++)  //电平翻转2次为一个脉冲，所以旋转一周需要3200个脉冲，
	{		
		Delay_us(300);                   //高低电平持续时间，脉冲频率
		GPIOD->ODR ^= GPIO_Pin_15;  				 //翻转输出电平
	}	
}
void Stepper_ON(void)
{
	GPIO_SetBits(GPIOD, GPIO_Pin_14);//步进电机A使能
	GPIO_SetBits(GPIOD, GPIO_Pin_11);//步进电机B使能
}
void Stepper_OFF(void)
{
	GPIO_ResetBits(GPIOD, GPIO_Pin_14);//步进电机A失能
	GPIO_ResetBits(GPIOD, GPIO_Pin_11);//步进电机B失能
}


