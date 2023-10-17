#ifndef SYS_H
#define SYS_H
#include <iostream>
#include <sys/time.h>

struct PIDINFO
{
    float _uORB_PID__Roll_Input = 0;
    float _uORB_PID_Pitch_Input = 0;
    float _uORB_PID_Yaw_Input = 0;
    float _uORB_RC_Out_Roll = 0;
    float _uORB_RC_Out_Pitch = 0;
    float _uORB_RC_Out_Yaw = 0;
    float _uORB_PID_AngleRate_Roll = 0;
    float _uORB_PID_AngleRate_Pitch = 0;
    float _uORB_Leveling__Roll = 0;
    float _uORB_Leveling__Pitch = 0;
    float _uORB_Leveling__Yaw = 0;

    float _uORB_PID_GYaw_Output = 0;
    float _flag_PID_AngleRate___Yaw_Gain = 0;

    float _uORB_PID_I_Last_Value__Roll = 0;
    float _uORB_PID_I_Last_Value__Pitch = 0;
    float _uORB_PID_D_Last_Value___Yaw = 0;

    float _uORB_PID_D_Last_Value__Roll = 0;
    float _uORB_PID_D_Last_Value__Pitch = 0;
    float _uORB_PID_I_Last_Value___Yaw = 0;

    float _flag_PID_AngleRate_Roll_Gain = 0;
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

    float _flag_PID_P___Yaw_Gain = 0;
    float _flag_PID_I___Yaw_Gain = 0;
    float _flag_PID_D___Yaw_Gain = 0;
    float _flag_PID_I___Yaw_Max__Value = 0;

    float _flag_PID_RCAngle__Yaw_Gain = 0;
} PF;

struct TaskThread
{
    float UpdateFreq = 1000000.0 / 4000.0;
    float _Tmp_IMUAttThreadLast = 0;
    float _Tmp_IMUAttThreadDT = 0;

} TF;

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

int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((tv.tv_sec * (uint64_t)1000000 + tv.tv_usec));
}

#endif