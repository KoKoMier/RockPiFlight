#include "../pid/pid.hpp"
#include <iostream>

extern int pwmValue[4];
extern float Angle_Pitch;
extern float Angle_Roll;
extern float Angle_Yaw;
extern float mpu_6500_GX; // x轴角速度
extern float mpu_6500_GY; // y轴角速度
extern float mpu_6500_GZ; // z轴角速度

using namespace std;

// typedef struct input
// {
    
// };


class CONTROL
{
private:
    //pid结构体
    PidTypeDef angle_pitch_pid;
    PidTypeDef angle_roll_pid;
    PidTypeDef angle_yaw_pid;

    PidTypeDef gyro_pitch_pid;
    PidTypeDef gyro_roll_pid;
    PidTypeDef gyro_yaw_pid;

    double angle_pitch;//pitch角度
    double angle_roll;//roll角度
    double angle_yaw;//yaw角度

    double gyro_pitch;//pitch角速度
    double gyro_roll;//roll角速度
    double gyro_yaw;//yaw角速度

    double targat_angle_pitch;//pitch目标角度
    double targat_angle_roll;//roll目标角度
    double targat_angle_yaw;//yaw目标角度

    double pitch_output;//pitch输出量
    double roll_output;//roll输出量
    double yaw_output;//yaw输出量
    double throttle;//油门

    double output1;//电机1输出量
    double output2;//电机2输出量
    double output3;//电机3输出量
    double output4;//电机4输出量

    void motor_pwm(int pwm1, int pwm2, int pwm3, int pwm4);
    void data_update(void);
    void limit(double &temp, double min, double max);
    void limit(int &temp, int min, int max);
public:
    CONTROL();
    ~CONTROL();
    void control(void);
    void set_target_angle(double pitch_angle, double roll_angle, double yaw_angle);
    void set_output(int _output1, int _output2, int _output3, int _output4);
    void set_throttle(int _throttle);
};

CONTROL::CONTROL()
{
    pid_init(&angle_pitch_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 500.0);
    pid_init(&angle_roll_pid, POSITION_MODE, 1.0, 0.0, 0.0, 0.0, 500.0);
    pid_init(&angle_yaw_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);

    pid_init(&gyro_pitch_pid, POSITION_MODE, 0.0, 0.0, 0.0, 500.0, 2000.0);
    pid_init(&gyro_roll_pid, POSITION_MODE, 1.0, 0.01, 0.0, 500.0, 2000.0);
    pid_init(&gyro_yaw_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);

    angle_pitch = 0;
    angle_roll = 0;
    angle_yaw = 0;

    gyro_pitch = 0;
    gyro_roll = 0;
    gyro_yaw = 0;

    targat_angle_pitch = 0;
    targat_angle_roll = 0;
    targat_angle_yaw = 0;

    pitch_output = 0;
    roll_output = 0;
    yaw_output = 0;
    throttle = 1800;

    output1 = 0;
    output2 = 0;
    output3 = 0;
    output4 = 0;
}

CONTROL::~CONTROL()
{

}


void CONTROL::control(void)
{
    data_update();//更新数据

    // pitch_output = pid_cascade_control(&angle_pitch_pid, &gyro_pitch_pid, targat_angle_pitch, angle_pitch, gyro_pitch);
    roll_output = pid_cascade_control(&angle_roll_pid, &gyro_roll_pid, targat_angle_roll, angle_roll, gyro_roll);
    // yaw_output = pid_cascade_control(&angle_yaw_pid, &gyro_yaw_pid, targat_yaw_pitch, angle_yaw, gyro_yaw);

    if(throttle > 1900)
    {
        output1 = throttle - pitch_output + roll_output;
        output2 = throttle - pitch_output - roll_output;
        output3 = throttle + pitch_output + roll_output;
        output4 = throttle + pitch_output - roll_output;
    }
    else
    {
        output1 = throttle;
        output2 = throttle;
        output3 = throttle;
        output4 = throttle;
    }
    
    

    //限幅
    limit(output1, 1800, 2500);
    limit(output2, 1800, 2500);
    limit(output3, 1800, 2500);
    limit(output4, 1800, 2500);

    // cout << "angle_pitch = " << angle_pitch << endl;
    // cout << "angle_roll = " << angle_roll << endl;
    // cout << "gyro_pitch = " << gyro_pitch << endl;
    // cout << "gyro_roll = " << gyro_roll << endl;
    // cout << "pitch_output = " << pitch_output << endl;
    // cout << "roll_output = " << roll_output << endl;
    cout << "throttle = " << throttle << endl;
    cout << "output1 = " << output1 << endl;
    cout << "output2 = " << output2 << endl;
    cout << "output3 = " << output3 << endl;
    cout << "output4 = " << output4 << endl;
    cout << endl;

    motor_pwm(output1, output2, output3, output4);//输出pwm
}


void CONTROL::motor_pwm(int pwm1, int pwm2, int pwm3, int pwm4)
{
    limit(pwm1, 1800, 3200);
    limit(pwm2, 1800, 3200);
    limit(pwm3, 1800, 3200);
    limit(pwm4, 1800, 3200);
    pwmValue[0] = pwm1;
    pwmValue[1] = pwm2;
    pwmValue[2] = pwm3;
    pwmValue[3] = pwm4;
}

void CONTROL::data_update(void)
{
    angle_pitch = -Angle_Pitch;
    angle_roll = -Angle_Roll;
    angle_yaw = -Angle_Yaw;

    gyro_pitch = mpu_6500_GX;
    gyro_roll = mpu_6500_GY;
    gyro_yaw = mpu_6500_GZ;
}

void CONTROL::limit(double &temp, double min, double max)
{
    if (temp < min)
    {
        temp = min;
    }
    else if(temp > max)
    {
        temp = max;
    }
}

void CONTROL::limit(int &temp, int min, int max)
{
    if (temp < min)
    {
        temp = min;
    }
    else if(temp > max)
    {
        temp = max;
    }
}


void CONTROL::set_target_angle(double pitch_angle, double roll_angle, double yaw_angle)
{

}


void CONTROL::set_output(int _output1, int _output2, int _output3, int _output4)
{
    limit(_output1, 1800, 3122);
    limit(_output2, 1800, 3122);
    limit(_output3, 1800, 3122);
    limit(_output4, 1800, 3122);

    output1 = _output1;
    output2 = _output2;
    output3 = _output3;
    output4 = _output4;
}

void CONTROL::set_throttle(int _throttle)
{
    limit(_throttle, 1800, 2800);
    throttle = _throttle;
}

