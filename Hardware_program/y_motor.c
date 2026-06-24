#include "y_motor.h"


PID A_SpeedPID={0};
PID B_SpeedPID={0};
PID C_SpeedPID={0};
PID D_SpeedPID={0};
int Z_step=0;				//丝杆底部为0
int Y_step=0;				//滑台右侧为0

/****************************PID参数初始化**************************************
函数：  void PID_int(void)
参数：  void
说明：  PID每个环参数初始化
返回值：void
********************************************************************************/

void PID_int(void)
{
//高速250
	float SPEED_Kp = 5;
	float SPEED_Ki = 5.5;
	float SPEED_Kd = 5;
	A_SpeedPID.Kp=5;     //速度环PID参数（D车用，速度环2ms）
	A_SpeedPID.Ki=3;
	A_SpeedPID.Kd=2;
	A_SpeedPID.LastError=0;
	A_SpeedPID.LLastError=0;
	A_SpeedPID.iIncpid=0;
	A_SpeedPID.errormax=350;
	A_SpeedPID.errormin=-500;
	A_SpeedPID.Imax=3500;
	
	B_SpeedPID.Kp=5;       //速度环PID参数（D车用，速度环2ms） 
	B_SpeedPID.Ki=3;
	B_SpeedPID.Kd=2;
	B_SpeedPID.LastError=0;
	B_SpeedPID.LLastError=0;
	B_SpeedPID.iIncpid=0;
	B_SpeedPID.errormax=350;
	B_SpeedPID.errormin=-350;
	B_SpeedPID.Imax=3500;
	
	C_SpeedPID.Kp=5;       //速度环PID参数（D车用，速度环2ms） 
	C_SpeedPID.Ki=3;
	C_SpeedPID.Kd=2;
	C_SpeedPID.LastError=0;
	C_SpeedPID.LLastError=0;
	C_SpeedPID.iIncpid=0;
	C_SpeedPID.errormax=350;
	C_SpeedPID.errormin=-350;
	C_SpeedPID.Imax=3500;
	
	D_SpeedPID.Kp=5;//速度环PID参数（D车用，速度环2ms） 
	D_SpeedPID.Ki=3;
	D_SpeedPID.Kd=2;
	D_SpeedPID.LastError=0;
	D_SpeedPID.LLastError=0;
	D_SpeedPID.iIncpid=0;	
	D_SpeedPID.errormax=350;
	D_SpeedPID.errormin=-350;
	D_SpeedPID.Imax=3500;

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
函数名：IncPIDCalc(PID *sptr,int16 Setpoint,int16 Turepoint)
功  能：增量式PID控制
参  数：PID *sptr,int16 Setpoint,int16 Turepoint
返回值：int32 iIncpid
************************************************/
float IncPIDCalc(PID *sptr, float Setpoint,float Turepoint)
{
    float iError;
		float Errormax;
		float Errormin;
    //当前误差
    iError = Setpoint-Turepoint;      //偏差
    Errormax= sptr->errormax;
		Errormin= sptr->errormin;
		//增量计算
		if(iError>Errormax)
		{
			iError=Errormax;
		}
		else if(iError<Errormin)
		{
			iError=Errormin;
		}
		
		sptr->iIncpid = (float)((sptr->Kp)*iError
										-(sptr->Ki)*(sptr->LastError)
										+sptr->Kd*( sptr->LLastError));
		//printf("%d\n",iIncpid);
		if (sptr->iIncpid >sptr->Imax)
			sptr->iIncpid =sptr->Imax;
		if (sptr->iIncpid <-sptr->Imax)
			sptr->iIncpid =-sptr->Imax;
	
    //储存误差，用于下次计算
    sptr->LLastError = sptr->LastError;
    sptr->LastError = iError;
		
		//printf("%f\n",A_SpeedPID.LastError);

    return(sptr->iIncpid);		
}


/**
 * @简  述 电机PWM速度控制
 * @参  数 speed 电机转速数值，范围-3000~3000（极限范围-4200~4200） SpeedA,编码器读值
 * @返回值 无
 */
void MOTOR_A_SetSpeed_Close(float speed,float SpeedA)
{ 
	int16_t Atemp;
	float aim_speed_A = speed;
	float OUTPUT_SPEED_A = 0;
	float T_SpeedA =SpeedA;
	
	OUTPUT_SPEED_A = IncPIDCalc(&A_SpeedPID,aim_speed_A,T_SpeedA);//D车速度环（增量式）
	//OUTPUT_SPEED_A = IncPIDCalc(&A_SpeedPID,T_SpeedA,aim_speed_A);//D车速度环（增量式）
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
void MOTOR_B_SetSpeed_Close(float speed,float SpeedB)
{
	int16_t Btemp;
	float aim_speed_B = speed;
	float OUTPUT_SPEED_B = 0;
	float T_SpeedB =SpeedB;
	
	OUTPUT_SPEED_B = IncPIDCalc(&B_SpeedPID,aim_speed_B,T_SpeedB);//D车速度环（增量式）
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
void MOTOR_D_SetSpeed_Close(float speed,float SpeedD)
{
	int16_t Dtemp;
	float aim_speed_D = speed;
	float OUTPUT_SPEED_D = 0;
	float T_SpeedD = SpeedD;
	
	OUTPUT_SPEED_D = IncPIDCalc(&D_SpeedPID, aim_speed_D, T_SpeedD);//D车速度环（增量式）
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
void MOTOR_C_SetSpeed_Close(float speed,float SpeedC)
{
	int16_t Ctemp;
	float aim_speed_C = speed;
	float OUTPUT_SPEED_C = 0;
	float T_SpeedC = SpeedC;
	
	
	OUTPUT_SPEED_C = IncPIDCalc(&C_SpeedPID,aim_speed_C,T_SpeedC);//D车速度环（增量式）
	
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


