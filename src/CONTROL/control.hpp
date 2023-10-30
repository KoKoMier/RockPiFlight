#pragma once
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

    float mpu_6500_gz = 0;

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
    void PID_Calculate(float inputDataP, float inputDataI, float inputDataD, float &outputData, float &last_I_Data,
                       float &last_D_Data, float P_Gain, float I_Gain, float D_Gain, float I_Max);
    void PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData, float &last_I_Data,
                            float &last_D_Data, float P_Gain, float I_Gain, float D_Gain, float I_Max);
};

void CONTROL::data_update()
{
    angle_pitch = Angle_Pitch_Out;
    angle_roll = Angle_Roll_Out;
    angle_yaw = Angle_Yaw_Gyro;

    PF._uORB_PID_GYaw_Output = -int(mpu_6500_GZ);

    gyro_pitch = mpu_6500_GX_Now;
    gyro_roll = mpu_6500_GY_Now;
    gyro_yaw = mpu_6500_GZ_Now;

    PF._uORB_PID__Roll_Input = 0;
    PF._uORB_PID_Pitch_Input = 0;
    PF._uORB_PID_Yaw_Input = 0;

    PF._uORB_PID_D_Last_Value__Roll = 0;
    PF._uORB_PID_D_Last_Value__Pitch = 0;
    PF._uORB_PID_D_Last_Value___Yaw = 0;
    PF._uORB_PID_I_Last_Value__Roll = 0;
    PF._uORB_PID_I_Last_Value__Pitch = 0;
    PF._uORB_PID_I_Last_Value___Yaw = 0;
    PF._flag_PID_I__Roll_Gain = 0;
    PF._flag_PID_I__Roll_Max__Value = 2400;
    PF._flag_PID_I__Pitch_Gain = 0;
    PF._flag_PID_I__Pitch_Max__Value = 2400;

    PF._uORB_PID_AngleRate_Pitch = angle_pitch - 1;
    PF._flag_PID_RCAngle__Pitch_Gain = 0.8;
    PF._flag_PID_AngleRate_Pitch_Gain = 10.f;
    PF._flag_PID_P__Pitch_Gain = 0.85;
    PF._flag_PID_D__Pitch_Gain = 67.0;

    PF._uORB_PID_AngleRate_Roll = angle_roll + 0.6;
    PF._flag_PID_RCAngle__Roll_Gain = 0.8;
    PF._flag_PID_AngleRate_Roll_Gain = 10.f;
    PF._flag_PID_P__Roll_Gain = 0.8;
    PF._flag_PID_D__Roll_Gain = 52.0;

    PF._flag_PID_AngleRate___Yaw_Gain = 5;
    PF._flag_PID_RCAngle__Yaw_Gain = 1.f;
    PF._flag_PID_P___Yaw_Gain = 2.6;
    PF._flag_PID_I___Yaw_Gain = 0;
    PF._flag_PID_D___Yaw_Gain = 50;

    PF._uORB_RC_Out_Roll = crsf.original_roll;
    PF._uORB_RC_Out_Pitch = crsf.original_pitch;
    PF._uORB_RC_Out_Yaw = crsf.original_yaw;
}

void CONTROL::PID_Calculate(float inputDataP, float inputDataI, float inputDataD, float &outputData, float &last_I_Data,
                            float &last_D_Data, float P_Gain, float I_Gain, float D_Gain, float I_Max)
{

    outputData = P_Gain * inputDataP;
    outputData -= D_Gain * inputDataD;
    // std::cout << "inputDataD  " << D_Gain * inputDataD << "\r\n";
    last_I_Data += I_Gain * inputDataI;

    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;

    outputData += last_I_Data;
}

void CONTROL::PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData, float &last_I_Data,
                                 float &last_D_Data, float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
    // P caculate
    outputData = P_Gain * inputDataP;
    // D caculate
    outputData += D_Gain * inputDataD;
    // I caculate
    last_I_Data += inputDataI * I_Gain;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    // P_I_D Mix OUTPUT
    outputData += last_I_Data;
}

void CONTROL::control(void)
{
    TF._Tmp_IMUAttThreadDT = GetTimestamp() - TF._Tmp_IMUAttThreadLast;
    data_update();

    PF._uORB_PID__Roll_Input += PF._uORB_RC_Out_Roll * PF._flag_PID_RCAngle__Roll_Gain;
    PF._uORB_PID_Pitch_Input += PF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle__Pitch_Gain;
    PF._uORB_PID_Yaw_Input += PF._uORB_RC_Out_Yaw * PF._flag_PID_RCAngle__Yaw_Gain;
    // std::cout << "PF._uORB_RC_Out_Pitch" << PF._uORB_RC_Out_Pitch << "\r\n";
    // std::cout << "PF._uORB_PID_Pitch_Input" << PF._uORB_PID_Pitch_Input << "\r\n";

    float AngleEXPO__Roll = PF._uORB_PID_AngleRate_Roll * PF._flag_PID_AngleRate_Roll_Gain;
    float AngleEXPO_Pitch = PF._uORB_PID_AngleRate_Pitch * PF._flag_PID_AngleRate_Pitch_Gain;
    PF._uORB_PID__Roll_Input += AngleEXPO__Roll;
    PF._uORB_PID_Pitch_Input += AngleEXPO_Pitch;

    // std::cout << "PF._uORB_PID_AngleRate_Pitch" << PF._uORB_PID_AngleRate_Pitch << "\r\n";
    // std::cout << "AngleEXPO_Pitch" << AngleEXPO_Pitch << "\r\n";
    // std::cout << "PF.AngleEXPO_Pitch" << PF._uORB_PID_Pitch_Input << "\r\n";
    // std::cout << "PF._uORB_PID__Roll_Input" << PF._uORB_PID__Roll_Input << "\r\n";

    PF._uORB_PID__Roll_Input -= gyro_roll * 1.35;
    PF._uORB_PID_Pitch_Input -= gyro_pitch * 1.35;
    // std::cout << "gyro_roll  " << gyro_roll << "\r\n";
    // std::cout << "PF._uORB_PID__Roll_Input" << PF._uORB_PID__Roll_Input << "\r\n";

    // Roll pid
    float ROLLDInput = gyro_roll - gyro_roll_last;
    gyro_roll_last = gyro_roll;
    float ROLLITERM = PF._uORB_PID__Roll_Input;
    float ROLLDTERM = ROLLDInput;
    // std::cout << "(TF._Tmp_IMUAttThreadDT / TF.UpdateFreq)" << (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq) << "\r\n";
    // std::cout << "ROLLDTERM" << ROLLDTERM << "\r\n";

    PID_Calculate(PF._uORB_PID__Roll_Input, ROLLITERM * (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq), ROLLDTERM / (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq),
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
    PID_Calculate(PF._uORB_PID_Pitch_Input, PITCHITERM * (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq), PITCHDTERM / (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq),
                  PF._uORB_Leveling__Pitch, PF._uORB_PID_I_Last_Value__Pitch, PF._uORB_PID_D_Last_Value__Pitch,
                  PF._flag_PID_P__Pitch_Gain,
                  PF._flag_PID_I__Pitch_Gain,
                  PF._flag_PID_D__Pitch_Gain,
                  PF._flag_PID_I__Pitch_Max__Value);

    if (PF._uORB_RC_Out_Yaw < 5 && PF._uORB_RC_Out_Yaw > -5)
    {
        PF._uORB_RC_Out_Yaw = 0;
    }
    else
    {
        PF._uORB_RC_Out_Yaw = PF._uORB_RC_Out_Yaw;
    }
    // Yaw pid
    PID_CaculateExtend((((PF._uORB_PID_GYaw_Output + PF._uORB_RC_Out_Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain),
                       ((((PF._uORB_PID_GYaw_Output + PF._uORB_RC_Out_Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) * (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq)),
                       ((((PF._uORB_PID_GYaw_Output) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) / (TF._Tmp_IMUAttThreadDT / TF.UpdateFreq)),
                       PF._uORB_Leveling__Yaw, PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
                       PF._flag_PID_P___Yaw_Gain, PF._flag_PID_I___Yaw_Gain, PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);

    // PF._uORB_Leveling__Yaw = 5 * angle_yaw + crsf.original_yaw;
    // std::cout << "PF._uORB_PID_GYaw_Output" << PF._uORB_PID_GYaw_Output << "\r\n";
    // std::cout << "PF.mpu_6500_GZ" << mpu_6500_GZ << "\r\n";
    // std::cout << "PF._uORB_Leveling__Yaw" << PF._uORB_Leveling__Yaw << "\r\n";

    // std::cout << "original_throttle" << crsf.original_throttle << "\r\n";
    // std::cout << "original_yaw" << crsf.original_yaw << "\r\n";
    // std::cout << "original_pitch" << crsf.original_pitch << "\r\n";
    // std::cout << "original_roll" << crsf.original_roll << "\r\n";
    // std::cout << "original_key1" << crsf.original_key1 << "\r\n";
    // std::cout << "original_key2" << crsf.original_key2 << "\r\n";
    // std::cout << "original_key3" << crsf.original_key3 << "\r\n";
    // std::cout << "original_key3" << crsf.original_key4 << "\r\n";

    // std::cout << "PF._uORB_Leveling__Roll" << PF._uORB_Leveling__Roll << "\r\n";
    // std::cout << "PF._uORB_Leveling__Yaw" << PF._uORB_Leveling__Yaw << "\r\n";
    // std::cout << "gyro_yaw" << gyro_yaw << "\r\n";
    pwmValue[0] = 1800 + crsf.original_throttle - PF._uORB_Leveling__Pitch + PF._uORB_Leveling__Roll + PF._uORB_Leveling__Yaw;
    pwmValue[1] = 1800 + crsf.original_throttle - PF._uORB_Leveling__Pitch - PF._uORB_Leveling__Roll - PF._uORB_Leveling__Yaw;
    pwmValue[2] = 1800 + crsf.original_throttle + PF._uORB_Leveling__Pitch + PF._uORB_Leveling__Roll - PF._uORB_Leveling__Yaw;
    pwmValue[3] = 1800 + crsf.original_throttle + PF._uORB_Leveling__Pitch - PF._uORB_Leveling__Roll + PF._uORB_Leveling__Yaw;

    // std::cout << "pwmValue[0]:" << pwmValue[0] << "\r\n";
    // std::cout << "pwmValue[1]:" << pwmValue[1] << "\r\n";
    // std::cout << "pwmValue[2]:" << pwmValue[2] << "\r\n";
    // std::cout << "pwmValue[3]:" << pwmValue[3] << "\r\n";
    std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(1) << PF._uORB_PID_AngleRate_Pitch << "\r\n";
    std::cout << "Angle_Roll:" << std::fixed << std::setprecision(1) << PF._uORB_PID_AngleRate_Roll << "\r\n";
    // std::cout << "------------------------------------"
    //           << "\r\n";

    TF._Tmp_IMUAttThreadLast = GetTimestamp();
}
