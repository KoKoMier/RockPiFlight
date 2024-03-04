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
        config.MPU_Flip__Roll = SF._flag_MPU_Flip__Roll;
        config.MPU_Flip_Pitch = SF._flag_MPU_Flip_Pitch;
        config.MPU_Flip___Yaw = SF._flag_MPU_Flip___Yaw;

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

void RockPiAPMAPI::RockPiAPM::IMUSensorsTaskReg()
{
    TF.IMUFlow.reset(new FlowThread(
        [&]
        {
            SF._uORB_MPU_Data = DF.MPUDevice->MPUSensorsDataGet();
            //============Online Catlibration======================================//
            {
                if (AF._flag_MPUCalibrating == 1)
                {
                }
            }
        },
        TF._flag_Sys_CPU_Asign, TF._flag_IMUFlowFreq));
}

void RockPiAPMAPI::RockPiAPM::ControllerTaskReg()
{
    TF.RTXFlow.reset(new FlowThread(
        [&] {

        },
        TF._flag_Sys_CPU_Asign, TF._flag_RTXFlowFreq));
}
void RockPiAPMAPI::RockPiAPM::ESCUpdateTaskReg()
{
    TF.ESCFlow.reset(new FlowThread(
        [&]
        {
            DF.I2CLock.lock();
            if (AF._flag_ESC_ARMED)
            {
                DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._Flag_Lock_Throttle);
                DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._Flag_Lock_Throttle);
                DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._Flag_Lock_Throttle);
                DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._Flag_Lock_Throttle);
            }
            if (!AF._flag_ESC_ARMED)
            {
                DF.ESCDevice->ESCUpdate(EF._flag_A1_Pin, EF._uORB_A1_Speed);
                DF.ESCDevice->ESCUpdate(EF._flag_A2_Pin, EF._uORB_A2_Speed);
                DF.ESCDevice->ESCUpdate(EF._flag_B1_Pin, EF._uORB_B1_Speed);
                DF.ESCDevice->ESCUpdate(EF._flag_B2_Pin, EF._uORB_B2_Speed);
            }
            DF.I2CLock.unlock();
        },
        TF._flag_Sys_CPU_Asign, TF._flag_ESCFlowFreq));
}

void RockPiAPMAPI::RockPiAPM::TaskThreadBlock()
{
    while (true)
    {
        std::cout << "SF._uORB_Real__Roll" << SF._uORB_MPU_Data._uORB_Real__Roll
                  << "\r\n";
        std::cout << "SF._uORB_Real_Pitch" << SF._uORB_MPU_Data._uORB_Real_Pitch
                  << "\r\n";
        std::cout << "======================================"
                  << "\r\n";
        usleep(10000);
    }
}
void RockPiAPMAPI::RockPiAPM::RockPiAPMStartUp()
{
    IMUSensorsTaskReg();

    ControllerTaskReg();

    ESCUpdateTaskReg();
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
    //==============================================================Sensors config==/
    SF._flag_MPU_Flip__Roll = APMInit.SC._flag_MPU_Flip__Roll;
    SF._flag_MPU_Flip_Pitch = APMInit.SC._flag_MPU_Flip_Pitch;
    SF._flag_MPU_Flip___Yaw = APMInit.SC._flag_MPU_Flip___Yaw;
    SF._flag_MPU_Accel_Cali[MPUAccelCaliX] = APMInit.SC._flag_MPU9250_A_X_Cali;
    SF._flag_MPU_Accel_Cali[MPUAccelCaliY] = APMInit.SC._flag_MPU9250_A_Y_Cali;
    SF._flag_MPU_Accel_Cali[MPUAccelCaliZ] = APMInit.SC._flag_MPU9250_A_Z_Cali;
    SF._flag_MPU_Accel_Cali[MPUAccelScalX] = APMInit.SC._flag_MPU9250_A_X_Scal;
    SF._flag_MPU_Accel_Cali[MPUAccelScalY] = APMInit.SC._flag_MPU9250_A_Y_Scal;
    SF._flag_MPU_Accel_Cali[MPUAccelScalZ] = APMInit.SC._flag_MPU9250_A_Z_Scal;
}

int RockPiAPMAPI::RockPiAPM::GetTimestamp()
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - TF._flag_SystemStartUp_Time);
}
