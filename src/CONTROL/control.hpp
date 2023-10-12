#ifndef CONTROL_H
#define CONTROL_H
#include <iostream>
#include "../SYS/sys.hpp"
#include "../MPU6500/MPU6500.hpp"

crsf_data crsf;
extern int pwmValue[4];

class CONTROL
{
private:
    float angle_pitch = 0; // pitch角度
    float angle_roll = 0;  // roll角度
    float angle_yaw = 0;   // yaw角度

    float gyro_pitch = 0; // pitch角速度
    float gyro_roll = 0;  // roll角速度
    float gyro_yaw = 0;   // yaw角速度

    float gyro_roll_last = 0;
    float gyro_pitch_last = 0;

    float targat_angle_pitch = 0; // pitch目标角度
    float targat_angle_roll = 0;  // roll目标角度
    float targat_angle_yaw = 0;   // yaw目标角度

    float pitch_output = 0; // pitch输出量
    float roll_output = 0;  // roll输出量
    float yaw_output = 0;   // yaw输出量

    float output1 = 0; // 电机1输出量
    float output2 = 0; // 电机2输出量
    float output3 = 0; // 电机3输出量
    float output4 = 0; // 电机4输出量

public:
    void control(void);
    void data_update(void);
};

void CONTROL::data_update()
{
    angle_pitch = Angle_Pitch_Out;
    angle_roll = Angle_Roll_Out;
    angle_yaw = Angle_Yaw_Gyro;

    gyro_pitch = mpu_6500_GX_Now;
    gyro_roll = mpu_6500_GY_Now;
    gyro_yaw = mpu_6500_GZ_Now;
}

void CONTROL::control(void)
{
    data_update();
    PF._uORB_PID__Roll_Input = 0;
    PF._uORB_PID_Pitch_Input = 0;

    PF._uORB_PID_D_Last_Value__Roll = 0;
    PF._uORB_PID_D_Last_Value__Pitch = 0;
    PF._uORB_PID_D_Last_Value___Yaw = 0;
    PF._uORB_PID_I_Last_Value__Roll = 0;
    PF._uORB_PID_I_Last_Value__Pitch = 0;
    PF._uORB_PID_I_Last_Value___Yaw = 0;
    PF._flag_PID_P__Roll_Gain = 0;
    PF._flag_PID_I__Roll_Gain = 0;
    PF._flag_PID_D__Roll_Gain = 0;
    PF._flag_PID_I__Roll_Max__Value = 0;
    PF._flag_PID_P__Pitch_Gain = 0;
    PF._flag_PID_I__Pitch_Gain = 0;
    PF._flag_PID_D__Pitch_Gain = 0;
    PF._flag_PID_I__Pitch_Max__Value = 0;

    PF._uORB_RC_Out_Roll = crsf.original_roll;
    PF._uORB_RC_Out_Pitch = crsf.original_pitch;

    PF._uORB_PID__Roll_Input -= PF._uORB_RC_Out_Roll * PF._flag_PID_RCAngle__Roll_Gain;
    PF._uORB_PID_Pitch_Input -= PF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle__Pitch_Gain;

    float AngleEXPO__Roll = PF._uORB_PID_AngleRate_Roll * PF._flag_PID_AngleRate__Roll_Gain;
    float AngleEXPO_Pitch = PF._uORB_PID_AngleRate_Pitch * PF._flag_PID_AngleRate_Pitch_Gain;
    PF._uORB_PID__Roll_Input += AngleEXPO__Roll;
    PF._uORB_PID_Pitch_Input += AngleEXPO_Pitch;

    PF._uORB_PID__Roll_Input += gyro_roll;
    PF._uORB_PID_Pitch_Input += gyro_pitch;

    // Roll pid
    float ROLLDInput = gyro_roll - gyro_roll_last;
    gyro_roll_last = gyro_roll;
    float ROLLITERM = PF._uORB_PID__Roll_Input;
    float ROLLDTERM = ROLLDInput;
    PID_Calculate(PF._uORB_PID__Roll_Input, ROLLITERM, ROLLDTERM,
                  PF._uORB_Leveling__Roll, PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
                  PF._flag_PID_P__Roll_Gain,
                  PF._flag_PID_I__Roll_Gain,
                  PF._flag_PID_D__Roll_Gain,
                  PF._flag_PID_I__Roll_Max__Value);

    // Pitch pid
    float PITCHDInput = gyro_pitch - gyro_pitch_last;
    gyro_pitch_last = gyro_pitch;
    float PITCHITERM = PF._uORB_PID_Pitch_Input;
    float PITCHDTERM = PITCHDInput;
    PID_Calculate(PF._uORB_PID_Pitch_Input, PITCHITERM, PITCHDTERM,
                  PF._uORB_Leveling__Pitch, PF._uORB_PID_I_Last_Value__Pitch, PF._uORB_PID_D_Last_Value__Pitch,
                  PF._flag_PID_P__Pitch_Gain,
                  PF._flag_PID_I__Pitch_Gain,
                  PF._flag_PID_D__Pitch_Gain,
                  PF._flag_PID_I__Pitch_Max__Value);

    // std::cout << "original_throttle" << crsf.original_throttle << "\r\n";
    // std::cout << "original_yaw" << crsf.original_yaw << "\r\n";
    // std::cout << "original_pitch" << crsf.original_pitch << "\r\n";
    // std::cout << "original_roll" << crsf.original_roll << "\r\n";
    // std::cout << "original_key1" << crsf.original_key1 << "\r\n";
    // std::cout << "original_key2" << crsf.original_key2 << "\r\n";
    // std::cout << "original_key3" << crsf.original_key3 << "\r\n";
    // std::cout << "original_key3" << crsf.original_key4 << "\r\n";

    pwmValue[0] = 1800 + crsf.original_throttle - PF._uORB_Leveling__Pitch;
    pwmValue[1] = 1800 + crsf.original_throttle - PF._uORB_Leveling__Pitch;
    pwmValue[2] = 1800 + crsf.original_throttle + PF._uORB_Leveling__Pitch;
    pwmValue[3] = 1800 + crsf.original_throttle + PF._uORB_Leveling__Pitch;

    std::cout << "pwmValue[0]" << pwmValue[0] << "\r\n";
    std::cout << "pwmValue[1]" << pwmValue[1] << "\r\n";
    std::cout << "pwmValue[2]" << pwmValue[2] << "\r\n";
    std::cout << "pwmValue[3]" << pwmValue[3] << "\r\n";
    // std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(1) << angle_pitch << "\r\n";
    // std::cout << "Angle_Roll:" << std::fixed << std::setprecision(1) << angle_roll << "\r\n";
    std::cout << "------------------------------------"
              << "\r\n";
}

#endif