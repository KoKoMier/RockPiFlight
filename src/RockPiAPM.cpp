#include "RockPiAPM.hpp"

int RockPiAPMAPI::RockPiAPM::RockPiAPMInit(APMSettinngs APMInit)
{

    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        TF._flag_SystemStartUp_Time = ((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000)));
    }
    ConfigReader(APMInit);

    //------------------------------------------------------------------------------------//
    {
        DF.ESCDevice.reset(new ESCGenerator(EF.ESCControllerType, DF.I2CDevice.c_str(), I2CPCA_ADDR, EF.ESCPLFrequency));
    }
    //------------------------------------------------------------------------------------//
    {
        MPUConfig config;
        DF.MPUDevice.reset(new RPiMPU6500(config));
        DF.MPUDevice->MPUCalibration(SF._flag_MPU_Accel_Cali);
    }
    return 0;
}

int RockPiAPMAPI::RockPiAPM::APMCalibrator(int controller, int action, int input, double *data)
{
    if (controller == ESCCalibration)
    {
        if (action == CaliESCStart)
        {
            sleep(5);
            DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._Flag_Max__Throttle);
            DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._Flag_Max__Throttle);
            DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._Flag_Max__Throttle);
            DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._Flag_Max__Throttle);
            sleep(15);
            DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._Flag_Lock_Throttle);
            DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._Flag_Lock_Throttle);
            DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._Flag_Lock_Throttle);
            DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._Flag_Lock_Throttle);
            return 0;
        }
        else if (action == CaliESCUserDefine)
        {
            DF.ESCDevice->ESCUpdate((int)data[0], input);
            return 1;
        }
    }
    else if (controller == ACCELCalibration)
    {
        DF.MPUDevice->MPUAccelCalibration(action, data);
    }

    return 0;
}

void RockPiAPMAPI::RockPiAPM::ConfigReader(APMSettinngs APMInit)
{
    //==========================================================Device Type=======/
    DF.I2CDevice = APMInit.DC.__I2CDevice;
    //==========================================================ESC config=========/
    EF._flag_A1_Pin = APMInit.OC._flag_A1_Pin;
    EF._flag_A2_Pin = APMInit.OC._flag_A2_Pin;
    EF._flag_B1_Pin = APMInit.OC._flag_B1_Pin;
    EF._flag_B2_Pin = APMInit.OC._flag_B2_Pin;
    EF.ESCPLFrequency = APMInit.OC.ESCPLFrequency;
    EF.ESCControllerType = (GeneratorType)APMInit.OC.ESCControllerType;
}

int RockPiAPMAPI::RockPiAPM::GetTimestamp()
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - TF._flag_SystemStartUp_Time);
}
