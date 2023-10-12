#include <iostream>

struct PIDINFO
{
    float _uORB_PID__Roll_Input = 0;
    float _uORB_PID_Pitch_Input = 0;
    float _uORB_RC_Out_Roll = 0;
    float _uORB_RC_Out_Pitch = 0;
    float _uORB_PID_AngleRate_Roll = 0;
    float _uORB_PID_AngleRate_Pitch = 0;

    float _uORB_Leveling__Roll = 0;
    float _uORB_Leveling__Pitch = 0;
    float _uORB_Leveling__Yaw = 0;

    float _uORB_PID_I_Last_Value__Roll = 0;
    float _uORB_PID_I_Last_Value__Pitch = 0;
    float _uORB_PID_D_Last_Value___Yaw = 0;
    
    float _uORB_PID_D_Last_Value__Roll = 0;
    float _uORB_PID_D_Last_Value__Pitch = 0;
    float _uORB_PID_I_Last_Value___Yaw = 0;

    float _flag_PID_AngleRate__Roll_Gain = 0;
    float _flag_PID_RCAngle__Roll_Gain = 0;
    
    float _flag_PID_AngleRate_Pitch_Gain = 0;
    float _flag_PID_RCAngle__Pitch_Gain = 0;

    float _flag_PID_P__Roll_Gain = 0;
    float _flag_PID_I__Roll_Gain = 0;
    float _flag_PID_D__Roll_Gain = 0;
    float _flag_PID_I__Roll_Max__Value = 0;

    float _flag_PID_P__Pitch_Gain = 0;
    float _flag_PID_I__Pitch_Gain = 0;
    float _flag_PID_D__Pitch_Gain = 0;
    float _flag_PID_I__Pitch_Max__Value = 0;
} PF;

typedef struct
{
    float original_pitch = 0;
    float original_roll = 0;
    float original_yaw = 0;
    float original_throttle = 0;

    int original_key1 = 0;
    int original_key2 = 0;
    int original_key3 = 0;
    int original_key4 = 0;

} crsf_data;

void PID_Calculate(float inputDataP, float inputDataI, float inputDataD, float outputData, float last_I_Data,
                   float last_D_Data, float P_Gain, float I_Gain, float D_Gain, float I_Max)
{

    outputData = P_Gain * inputDataP;
    outputData += D_Gain * inputDataD;
    last_I_Data += I_Gain * inputDataI;

    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;

    outputData += last_I_Data;
}
