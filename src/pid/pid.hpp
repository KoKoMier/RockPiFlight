#include "stdio.h"

#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

typedef enum
{
  POSITION_MODE = 0,
  INCREMENT_MODE =1,
}PID_MODE;

typedef struct
{
	PID_MODE mode;
	
	double Kp;
	double Ki;
	double Kd;

	double max_out; 
	double max_iout;

	double measure;
	double target;

	double Pout;
	double Iout;
	double Dout;
	double PID_out;
	
	double Dbuf[3];  
	double error[3]; 
} PidTypeDef;


void pid_init(PidTypeDef *pid, PID_MODE mode, const double Kp, const double Ki, const double Kd, const double max_iout, const double max_out);
double pid_control(PidTypeDef * pid, double target, double measure);
double pid_cascade_control(PidTypeDef  * pid_outer, PidTypeDef * pid_inside, double pid_outer_target, double pid_outer_mersure, double pid_inside_mersure);
void pid_clear(PidTypeDef *pid);


/**
  * @brief     PID结构体初始化函数
  * @param     pid	PID结构体指针		mode	PID模式		Kp，Ki，Kd	PID系数		max_iout	微分项最大值		max_out	PID输出最大值
  * @retval    None
  * @return    None
  * @note      None
  */
void pid_init(PidTypeDef *pid, PID_MODE mode, const double Kp, const double Ki, const double Kd, const double max_iout, const double max_out)
{
	pid->mode = mode;

	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	
	pid->max_iout = max_iout;
	pid->max_out = max_out;
	
	pid->measure = 0.0f;
	pid->target = 0.0f;
	
	pid->Pout = 0.0f;
	pid->Iout = 0.0f;
	pid->Dout = 0.0f;
	pid->PID_out = 0.0f;
	
	pid->Dbuf[0] = 0.0f;
	pid->Dbuf[1] = 0.0f;
	pid->Dbuf[2] = 0.0f;
	pid->error[0] = 0.0f;
	pid->error[1] = 0.0f;
	pid->error[2] = 0.0f;
}


/**
  * @brief     PID运算函数
  * @param     pid	PID结构体指针		target	目标值		measure 测量值
  * @retval    double
  * @return    PID输出值
  * @note      None
  */
double pid_control(PidTypeDef * pid, double target, double measure)
{
	if (pid == NULL)
	{
		return 0.0f;
	}
	pid->error[2] = pid->error[1];
	pid->error[1] = pid->error[0];
	pid->measure = measure;
	pid->target = target;
	pid->error[0] = target - measure;
	if (pid->mode == POSITION_MODE)
	{
		pid->Pout = pid->Kp * pid->error[0];
		pid->Iout += pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		LimitMax(pid->Iout, pid->max_iout);
		pid->PID_out = pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->PID_out, pid->max_out);
	}
	else if (pid->mode == INCREMENT_MODE)
	{
		pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
		pid->Iout = pid->Ki * pid->error[0];
		pid->Dbuf[2] = pid->Dbuf[1];
		pid->Dbuf[1] = pid->Dbuf[0];
		pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
		pid->Dout = pid->Kd * pid->Dbuf[0];
		LimitMax(pid->Iout, pid->max_iout);
		pid->PID_out += pid->Pout + pid->Iout + pid->Dout;
		LimitMax(pid->PID_out, pid->max_out);
	}
	return pid->PID_out;
}


/**
  * @brief     串级PID运算函数
  * @param     pid	外环PID结构体指针		pid	内环PID结构体指针		pid_outer_target	外环目标值		pid_outer_mersure 外环测量值		pid_inside_mersure 内环测量值
  * @retval    double
  * @return    PID输出值
  * @note      None
  */
double pid_cascade_control(PidTypeDef  * pid_outer, PidTypeDef * pid_inside, double pid_outer_target, double pid_outer_mersure, double pid_inside_mersure)
{
	if (pid_outer == NULL)
	{
		return 0.0f;
	}
	if (pid_inside == NULL)
	{
		return 0.0f;
	}
	return pid_control(pid_inside, pid_control(pid_outer, pid_outer_target, pid_outer_mersure), pid_inside_mersure);
}


/**
  * @brief     PID结构体清除函数
  * @param     pid	PID结构体指针
  * @retval    None
  * @return    None
  * @note      None
  */
void pid_clear(PidTypeDef *pid)
{
  if (pid == NULL)
	{
		return;
	}
	pid->measure = 0.0f;
	pid->target = 0.0f;
	
	pid->Dbuf[0] = 0.0f;
	pid->Dbuf[0] = 0.0f;
	pid->Dbuf[0] = 0.0f;
	pid->error[0] = 0.0f;
	pid->error[0] = 0.0f;
	pid->error[0] = 0.0f;
	
	pid->Pout = 0.0f;
	pid->Iout = 0.0f;
	pid->Dout = 0.0f;
	pid->PID_out = 0.0f;
}
