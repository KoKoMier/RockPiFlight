#include "RockPiAPM.hpp"

int RockPiAPMAPI::RockPiAPM::RockPiAPMInit(APMSettinngs APMInit)
{
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        TF._flag_SystemStartUp_Time = ((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000)));
    }
    //------------------------------------------------------------------------------------//
    {
        pca9685PWMSetup("/dev/i2c-7",I2CPCA_ADDR,1526);
    }
}

int RockPiAPMAPI::RockPiAPM::APMCalibrator(int controller, int action, int input, double *data)
{
    if (controller == ESCCalibration)
    {
        if (action == CaliESCStart)
        {
            
        }
    }
    else
        ;
}

int RockPiAPMAPI::RockPiAPM::GetTimestamp()
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - TF._flag_SystemStartUp_Time);
}