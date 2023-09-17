#include "../pid/pid.hpp"
#include <iostream>

using namespace std;

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
public:
    CONTROL();
    ~CONTROL();
    void control(void);
    void set_target_angle(double pitch_angle, double roll_angle, double yaw_angle);
};

CONTROL::CONTROL()
{
    pid_init(&angle_pitch_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);
    pid_init(&angle_roll_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);
    pid_init(&angle_yaw_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);

    pid_init(&gyro_pitch_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);
    pid_init(&gyro_roll_pid, POSITION_MODE, 0.0, 0.0, 0.0, 0.0, 0.0);
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
    throttle = 0;

    output1 = 0;
    output2 = 0;
    output3 = 0;
    output4 = 0;
}

CONTROL::~CONTROL()
{

}

void CONTROL::motor_pwm(int pwm1, int pwm2, int pwm3, int pwm4)
{

}

void CONTROL::data_update(void)
{

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

void CONTROL::control(void)
{
    // pitch_output = pid_cascade_control(&angle_pitch_pid, &gyro_pitch_pid, targat_angle_pitch, angle_pitch, gyro_pitch);
    // roll_output = pid_cascade_control(&angle_roll_pid, &gyro_roll_pid, targat_roll_pitch, angle_roll, gyro_roll);
    // yaw_output = pid_cascade_control(&angle_yaw_pid, &gyro_yaw_pid, targat_yaw_pitch, angle_yaw, gyro_yaw);

    // output1 = throttle + pitch_output + roll_output;
    // output2 = throttle + pitch_output + roll_output;
    // output3 = throttle + pitch_output + roll_output;
    // output4 = throttle + pitch_output + roll_output;
    output1 = 1000;

    //限幅
    limit(output1, 1800, 3122);
    limit(output2, 1800, 3122);
    limit(output3, 1800, 3122);
    limit(output4, 1800, 3122);

    cout << "output1 = " << output1 << endl;

    motor_pwm(output1, output2, output3, output4);//输出pwm
}

void CONTROL::set_target_angle(double pitch_angle, double roll_angle, double yaw_angle)
{

}


