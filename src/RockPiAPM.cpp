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
        config.MPUSPIChannel = DF.MPUDeviceSPI.c_str();

        DF.MPUDevice.reset(new RPiMPU6500(config));
        DF.MPUDevice->MPUCalibration(SF._flag_MPU_Accel_Cali);
    }
    //------------------------------------------------------------------------------------//
    {
        if (RF.RC_Type == RCIsIbus)
        {
        }
        else if (RF.RC_Type == RCIsSbus)
        {
        }
        else if (RF.RC_Type == RCIsCRSF)
        {
            DF.CRSFInit.reset(new CRSF(DF.RCDevice.c_str()));
        }
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
            //=====================================================================//
            AttitudeUpdate();
        },
        TF._flag_Sys_CPU_Asign, TF._flag_IMUFlowFreq));
}

void RockPiAPMAPI::RockPiAPM::ControllerTaskReg()
{
    TF.RTXFlow.reset(new FlowThread(
        [&]
        {
            if (RF.RC_Type == RCIsIbus)
            {
            }
            else if (RF.RC_Type == RCIsSbus)
            {
            }
            else if (RF.RC_Type == RCIsCRSF)
            {
                if (DF.CRSFInit->CRSFRead(RF._Tmp_RC_Data, 15000) > 0)
                {
                    for (size_t i = 0; i < 16; i++)
                    {
                        RF._uORB_RC_Channel_PWM[i] = DF.CRSFInit->rcToUs(RF._Tmp_RC_Data[i]);
                    }
                }
            }
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
    TF._flag_Block_Task_Running = true;
    while (TF._flag_Block_Task_Running)
    {
        DebugOutPut();
        SaftyCheck();
        usleep(10000);
    }
}
void RockPiAPMAPI::RockPiAPM::RockPiAPMStartUp()
{
    IMUSensorsTaskReg();

    ControllerTaskReg();

    ESCUpdateTaskReg();
}

void RockPiAPMAPI::RockPiAPM::AttitudeUpdate()
{
    {
        // IMU SaftyChecking---------------------------------------------------------//
        if (SF._uORB_MPU_Data._uORB_Real__Roll > 70 || SF._uORB_MPU_Data._uORB_Real__Roll < -70 ||
            SF._uORB_MPU_Data._uORB_Real_Pitch > 70 || SF._uORB_MPU_Data._uORB_Real_Pitch < -70)
        {
            ////////////////////////////////////////////////////////////////////////////////
        }
        SF._uORB_MPU_Data._uORB_Gryo_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch > PF._flag_PID_Rate_Limit ? PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo_Pitch;
        SF._uORB_MPU_Data._uORB_Gryo_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch < -1 * PF._flag_PID_Rate_Limit ? -1 * PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo_Pitch;
        SF._uORB_MPU_Data._uORB_Gryo__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll > PF._flag_PID_Rate_Limit ? PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo__Roll;
        SF._uORB_MPU_Data._uORB_Gryo__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll < -1 * PF._flag_PID_Rate_Limit ? -1 * PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo__Roll;
        
    }
}

void RockPiAPMAPI::RockPiAPM::PID_Caculate(float inputData, float &outputData,
                                           float &last_I_Data, float &last_D_Data,
                                           float P_Gain, float I_Gain, float D_Gain, float I_Max)

{
    // P caculate
    outputData = P_Gain * inputData;
    // D caculate
    outputData += D_Gain * (inputData - last_D_Data);
    last_D_Data = inputData;
    // I caculate
    last_I_Data += I_Gain * inputData;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    // P_I_D Mix OUTPUT
    outputData += last_I_Data;
}

void RockPiAPMAPI::RockPiAPM::PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                                                 float &last_I_Data, float &last_D_Data,
                                                 float P_Gain, float I_Gain, float D_Gain, float I_Max)
{
    // P caculate
    outputData = P_Gain * inputDataP;
    // D caculate
    outputData += D_Gain * (inputDataD - last_D_Data);
    last_D_Data = inputDataD;
    // I caculate
    last_I_Data += inputDataI * I_Gain;
    if (last_I_Data > I_Max)
        last_I_Data = I_Max;
    if (last_I_Data < I_Max * -1)
        last_I_Data = I_Max * -1;
    // P_I_D Mix OUTPUT
    outputData += last_I_Data;
}

void RockPiAPMAPI::RockPiAPM::PID_CaculateHyper(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                                                float &last_I_Data, float &last_D_Data,
                                                float P_Gain, float I_Gain, float D_Gain, float I_Max)
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
void RockPiAPMAPI::RockPiAPM::RPiSingleAPMDeInit()
{
    TF._flag_Block_Task_Running = false;

    if (TF.IMUFlow)
        TF.IMUFlow->FlowStopAndWait();
    if (TF.RTXFlow)
        TF.RTXFlow->FlowStopAndWait();
    if (TF.ESCFlow)
        TF.ESCFlow->FlowStopAndWait();

    TF.IMUFlow.reset();
    TF.RTXFlow.reset();
    TF.ESCFlow.reset();

    //--------------------------------------------------------------------//
    usleep(5000);
    if (DF.ESCDevice.get() != nullptr)
        DF.ESCDevice->ESCClear(PCA9685_ALL_PIN);
    DF.ESCDevice.reset();
    DF.MPUDevice.reset();
    std::exit(EXIT_SUCCESS);
}

void RockPiAPMAPI::RockPiAPM::SaftyCheck()
{
    if (SystemSignal == SIGTERM || SystemSignal == SIGINT)
        RPiSingleAPMDeInit();
}

void RockPiAPMAPI::RockPiAPM::ConfigReader(APMSettinngs APMInit)
{
    //==========================================================Device Type=======/
    RF.RC_Type = RCIsCRSF;
    DF.I2CDevice = APMInit.DC.__I2CDevice;
    DF.RCDevice = "/dev/ttyS2";
    DF.MPUDeviceSPI = "/dev/spidev0.0";
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

void RockPiAPMAPI::RockPiAPM::DebugOutPut()
{
    std::cout << "SF._uORB_Real__Roll" << SF._uORB_MPU_Data._uORB_Real__Roll
              << "\r\n";
    std::cout << "SF._uORB_Real_Pitch" << SF._uORB_MPU_Data._uORB_Real_Pitch
              << "\r\n";
    std::cout << "SF._uORB_Gryo__Roll" << SF._uORB_MPU_Data._uORB_Gryo__Roll
              << "\r\n";
    std::cout << "SF._uORB_Gryo_Pitch" << SF._uORB_MPU_Data._uORB_Gryo_Pitch
              << "\r\n";
    // std::cout << "RF._uORB_RC_Channel_PWM[0]" << RF._uORB_RC_Channel_PWM[0] // roll 1000 - 2000
    //           << "\r\n";
    // std::cout << "RF._uORB_RC_Channel_PWM[1]" << RF._uORB_RC_Channel_PWM[1] // pitch 1000 - 2000
    //           << "\r\n";
    // std::cout << "RF._uORB_RC_Channel_PWM[2]" << RF._uORB_RC_Channel_PWM[2] // oil 1000 - 2000
    //           << "\r\n";
    // std::cout << "RF._uORB_RC_Channel_PWM[3]" << RF._uORB_RC_Channel_PWM[3] // yaw  1000 - 2000
    //           << "\r\n";
    // std::cout << "RF._uORB_RC_Channel_PWM[4]" << RF._uORB_RC_Channel_PWM[4] // stop
    //           << "\r\n";
    // std::cout << "RF._uORB_RC_Channel_PWM[5]" << RF._uORB_RC_Channel_PWM[5] // g z 1000 1500 2000
    //           << "\r\n";
    std::cout << "======================================"
              << "\r\n";
}
