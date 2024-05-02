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
    //------------------------------------------------------------------------------------//
    {
        pt1FilterInit(&DF.AngleRateLPF[0], PF._flag_Filter_AngleRate_CutOff, 0.f);
        pt1FilterInit(&DF.AngleRateLPF[1], PF._flag_Filter_AngleRate_CutOff, 0.f);
        pt1FilterInit(&DF.AngleRateLPF[2], PF._flag_Filter_AngleRate_CutOff, 0.f);
        pt1FilterInit(&DF.ItermFilterRoll, PF._flag_Filter_PID_I_CutOff, 0.f);
        pt1FilterInit(&DF.DtermFilterRoll, PF._flag_Filter_PID_D_ST1_CutOff, 0.f);
        pt1FilterInit(&DF.DtermFilterRollST2, PF._flag_Filter_PID_D_ST2_CutOff, 0.f);
        pt1FilterInit(&DF.ItermFilterPitch, PF._flag_Filter_PID_I_CutOff, 0.f);
        pt1FilterInit(&DF.DtermFilterPitch, PF._flag_Filter_PID_D_ST1_CutOff, 0.f);
        pt1FilterInit(&DF.DtermFilterPitchST2, PF._flag_Filter_PID_D_ST2_CutOff, 0.f);
        pt1FilterInit(&DF.IMUDtLPF, FILTERIMUDTLPFCUTOFF, 0.0f);
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
            if (SF._flag_MPU_Down == 1)
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
            if (SF._uORB_MPU_Data._uORB_Real__Roll < 2 && SF._uORB_MPU_Data._uORB_Real__Roll > -2 &&
                SF._uORB_MPU_Data._uORB_Real_Pitch < 4.5 && SF._uORB_MPU_Data._uORB_Real_Pitch > -4.5)
            {
                SF._flag_MPU_Down = 1;
            }
            {
                // RC Out Caculation
                RF._Tmp_RC_Out__Roll = (RF._uORB_RC_Channel_PWM[0] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv__Roll;
                RF._Tmp_RC_Out_Pitch = (RF._uORB_RC_Channel_PWM[1] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv_Pitch;
                RF._Tmp_RC_Out___Yaw = (RF._uORB_RC_Channel_PWM[3] - RF._flag_RC_Mid_PWM_Value) * RF._flag_RCIsReserv___Yaw;
                RF._Tmp_RC_Out_Throttle = RF._uORB_RC_Channel_PWM[2];
            }
            if (RF._uORB_RC_Channel_PWM[4] < 1200)
            {
                AF._flag_ESC_ARMED = true;
            }
            else if (RF._uORB_RC_Channel_PWM[4] > 1800)
            {
                AF._flag_ESC_ARMED = false;
            }
            else
            {
                AF._flag_ESC_ARMED = true;
            }
            // RC data out
            {
                RF._uORB_RC_Out__Roll = RF._Tmp_RC_Out__Roll;
                RF._uORB_RC_Out_Pitch = RF._Tmp_RC_Out_Pitch;
                RF._uORB_RC_Out___Yaw = RF._Tmp_RC_Out___Yaw;
                RF._uORB_RC_Out_Throttle = RF._Tmp_RC_Out_Throttle;
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
    TF._Tmp_IMUAttThreadDT = GetTimestamp() - TF._Tmp_IMUAttThreadLast;
    TF._uORB_IMUAttThreadDT = (int)pt1FilterApply4(&DF.IMUDtLPF, (int)TF._Tmp_IMUAttThreadDT, FILTERIMUDTLPFCUTOFF, 1.f / TF._flag_IMUFlowFreq);
    {

        // IMU SaftyChecking---------------------------------------------------------//
        if (SF._uORB_MPU_Data._uORB_Real__Roll > 70 || SF._uORB_MPU_Data._uORB_Real__Roll < -70 ||
            SF._uORB_MPU_Data._uORB_Real_Pitch > 70 || SF._uORB_MPU_Data._uORB_Real_Pitch < -70)
        {
            ////////////////////////////////////////////////////////////////////////////////
        }
        if (PF._flag_Filter_AngleRate_CutOff != 0)
            PF._uORB_PID_AngleRate__Roll = pt1FilterApply4(&DF.AngleRateLPF[0], SF._uORB_MPU_Data._uORB_Real__Roll,
                                                           PF._flag_Filter_AngleRate_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        else
            PF._uORB_PID_AngleRate__Roll = SF._uORB_MPU_Data._uORB_Real__Roll;
        //
        if (PF._flag_Filter_AngleRate_CutOff != 0)
            PF._uORB_PID_AngleRate_Pitch = pt1FilterApply4(&DF.AngleRateLPF[1], SF._uORB_MPU_Data._uORB_Real_Pitch,
                                                           PF._flag_Filter_AngleRate_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        else
            PF._uORB_PID_AngleRate_Pitch = SF._uORB_MPU_Data._uORB_Real_Pitch;
        //
        if (SF._flag_Filter_GYaw_CutOff != 0)
            PF._uORB_PID_GYaw_Output = pt1FilterApply4(&DF.AngleRateLPF[2], SF._uORB_MPU_Data._uORB_Gryo___Yaw,
                                                       SF._flag_Filter_GYaw_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        else
            PF._uORB_PID_GYaw_Output = SF._uORB_MPU_Data._uORB_Gryo___Yaw;

        // ---------------------------------------------------------------------//
        PF._uORB_PID__Roll_Input = 0;
        PF._uORB_PID_Pitch_Input = 0;
        PF._uORB_PID__Roll_Input += RF._uORB_RC_Out__Roll * PF._flag_PID_RCAngle__Roll_Gain;
        PF._uORB_PID_Pitch_Input += RF._uORB_RC_Out_Pitch * PF._flag_PID_RCAngle_Pitch_Gain;
        float AngleEXPO__Roll = PF._uORB_PID_AngleRate__Roll * PF._flag_PID_AngleRate__Roll_Gain;
        float AngleEXPO_Pitch = PF._uORB_PID_AngleRate_Pitch * PF._flag_PID_AngleRate_Pitch_Gain;
        PF._uORB_PID__Roll_Input += AngleEXPO__Roll;
        PF._uORB_PID_Pitch_Input += AngleEXPO_Pitch;

        // ---------------------------------------------------------------------//
        SF._uORB_MPU_Data._uORB_Gryo_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch > PF._flag_PID_Rate_Limit ? PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo_Pitch;
        SF._uORB_MPU_Data._uORB_Gryo_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch < -1 * PF._flag_PID_Rate_Limit ? -1 * PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo_Pitch;
        SF._uORB_MPU_Data._uORB_Gryo__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll > PF._flag_PID_Rate_Limit ? PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo__Roll;
        SF._uORB_MPU_Data._uORB_Gryo__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll < -1 * PF._flag_PID_Rate_Limit ? -1 * PF._flag_PID_Rate_Limit : SF._uORB_MPU_Data._uORB_Gryo__Roll;
        PF._uORB_PID__Roll_Input += SF._uORB_MPU_Data._uORB_Gryo__Roll;
        PF._uORB_PID_Pitch_Input += SF._uORB_MPU_Data._uORB_Gryo_Pitch;

        //--------------------------------------------------------------------------//
        // Roll PID
        float ROLLDInput = SF._uORB_MPU_Data._uORB_Gryo__Roll - PF._uORB_PID_D_Last_Value__Roll;
        PF._uORB_PID_D_Last_Value__Roll = SF._uORB_MPU_Data._uORB_Gryo__Roll;
        float ROLLITERM = PF._uORB_PID__Roll_Input;
        float ROLLDTERM = ROLLDInput;
        if (PF._flag_Filter_PID_I_CutOff)
            ROLLITERM = pt1FilterApply4(&DF.ItermFilterRoll, PF._uORB_PID__Roll_Input, PF._flag_Filter_PID_I_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        if (PF._flag_Filter_PID_D_ST1_CutOff)
            ROLLDTERM = pt1FilterApply4(&DF.DtermFilterRoll, ROLLDInput, PF._flag_Filter_PID_D_ST1_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        if (PF._flag_Filter_PID_D_ST2_CutOff)
            ROLLDTERM = pt1FilterApply4(&DF.DtermFilterRollST2, ROLLDTERM, PF._flag_Filter_PID_D_ST2_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));

        PID_CaculateHyper((PF._uORB_PID__Roll_Input),
                          (ROLLITERM * (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
                          (ROLLDTERM / (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
                          PF._uORB_Leveling__Roll, PF._uORB_PID_I_Last_Value__Roll, PF._uORB_PID_D_Last_Value__Roll,
                          (PF._flag_PID_P__Roll_Gain * PF._uORB_PID_TPA_Beta),
                          (PF._flag_PID_I__Roll_Gain * PF._uORB_PID_I_Dynamic_Gain),
                          (PF._flag_PID_D__Roll_Gain * PF._uORB_PID_TPA_Beta),
                          PF._flag_PID_I__Roll_Max__Value);
        if (PF._uORB_Leveling__Roll > PF._flag_PID_Level_Max)
            PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max;
        if (PF._uORB_Leveling__Roll < PF._flag_PID_Level_Max * -1)
            PF._uORB_Leveling__Roll = PF._flag_PID_Level_Max * -1;

        // Pitch PID
        float PITCHDInput = SF._uORB_MPU_Data._uORB_Gryo_Pitch - PF._uORB_PID_D_Last_Value_Pitch;
        PF._uORB_PID_D_Last_Value_Pitch = SF._uORB_MPU_Data._uORB_Gryo_Pitch;
        float PITCHITERM = PF._uORB_PID_Pitch_Input;
        float PITCHDTERM = PITCHDInput;
        if (PF._flag_Filter_PID_I_CutOff)
            PITCHITERM = pt1FilterApply4(&DF.ItermFilterPitch, PF._uORB_PID_Pitch_Input, PF._flag_Filter_PID_I_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        if (PF._flag_Filter_PID_D_ST1_CutOff)
            PITCHDTERM = pt1FilterApply4(&DF.DtermFilterPitch, PITCHDInput, PF._flag_Filter_PID_D_ST1_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        if (PF._flag_Filter_PID_D_ST2_CutOff)
            PITCHDTERM = pt1FilterApply4(&DF.DtermFilterPitchST2, PITCHDTERM, PF._flag_Filter_PID_D_ST2_CutOff, ((float)TF._uORB_IMUAttThreadDT / 1000000.f));
        PID_CaculateHyper((PF._uORB_PID_Pitch_Input),
                          (PITCHITERM * (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
                          (PITCHDTERM / (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)),
                          PF._uORB_Leveling_Pitch, PF._uORB_PID_I_Last_Value_Pitch, PF._uORB_PID_D_Last_Value_Pitch,
                          (PF._flag_PID_P_Pitch_Gain * PF._uORB_PID_TPA_Beta),
                          (PF._flag_PID_I_Pitch_Gain * PF._uORB_PID_I_Dynamic_Gain),
                          (PF._flag_PID_D_Pitch_Gain * PF._uORB_PID_TPA_Beta),
                          PF._flag_PID_I_Pitch_Max__Value);
        if (PF._uORB_Leveling_Pitch > PF._flag_PID_Level_Max)
            PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max;
        if (PF._uORB_Leveling_Pitch < PF._flag_PID_Level_Max * -1)
            PF._uORB_Leveling_Pitch = PF._flag_PID_Level_Max * -1;

        // Yaw PID
        PID_CaculateExtend((((PF._uORB_PID_GYaw_Output + RF._uORB_RC_Out___Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) * EF._flag_YAWOut_Reverse,
                           ((((PF._uORB_PID_GYaw_Output + RF._uORB_RC_Out___Yaw) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) * (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)) * EF._flag_YAWOut_Reverse,
                           ((((PF._uORB_PID_GYaw_Output) / 15.f) * PF._flag_PID_AngleRate___Yaw_Gain) / (TF._uORB_IMUAttThreadDT / PID_DT_DEFAULT)) * EF._flag_YAWOut_Reverse,
                           PF._uORB_Leveling___Yaw, PF._uORB_PID_I_Last_Value___Yaw, PF._uORB_PID_D_Last_Value___Yaw,
                           PF._flag_PID_P___Yaw_Gain, (PF._flag_PID_I___Yaw_Gain * PF._uORB_PID_I_Dynamic_Gain), PF._flag_PID_D___Yaw_Gain, PF._flag_PID_I___Yaw_Max__Value);

        //----------------------------------------------------------//
        EF._uORB_Total_Throttle = RF._uORB_RC_Out_Throttle;
        if (PF._uORB_Leveling___Yaw > PF._flag_PID_Level_Max)
            PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max;
        if (PF._uORB_Leveling___Yaw < PF._flag_PID_Level_Max * -1)
            PF._uORB_Leveling___Yaw = PF._flag_PID_Level_Max * -1;
        {
            EF._Tmp_B1_Speed = 0;
            EF._Tmp_A1_Speed = 0;
            EF._Tmp_A2_Speed = 0;
            EF._Tmp_B2_Speed = 0;

            EF._Tmp_B1_Speed = -PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
            EF._Tmp_A1_Speed = -PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;
            EF._Tmp_A2_Speed = +PF._uORB_Leveling__Roll + PF._uORB_Leveling_Pitch - PF._uORB_Leveling___Yaw;
            EF._Tmp_B2_Speed = +PF._uORB_Leveling__Roll - PF._uORB_Leveling_Pitch + PF._uORB_Leveling___Yaw;

            if (EF._uORB_Total_Throttle < EF._uORB_Dynamic_ThrottleMin)
                EF._uORB_Total_Throttle = EF._uORB_Dynamic_ThrottleMin;
            else if (EF._uORB_Total_Throttle > EF._uORB_Dynamic_ThrottleMax)
                EF._uORB_Total_Throttle = EF._uORB_Dynamic_ThrottleMax;
            EF._Tmp_B1_Speed += EF._uORB_Total_Throttle;
            EF._Tmp_A1_Speed += EF._uORB_Total_Throttle;
            EF._Tmp_A2_Speed += EF._uORB_Total_Throttle;
            EF._Tmp_B2_Speed += EF._uORB_Total_Throttle;

            EF._Tmp_A1_Speed = EF._Tmp_A1_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_A1_Speed;
            EF._Tmp_A2_Speed = EF._Tmp_A2_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_A2_Speed;
            EF._Tmp_B1_Speed = EF._Tmp_B1_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_B1_Speed;
            EF._Tmp_B2_Speed = EF._Tmp_B2_Speed < RF._flag_RC_Min_PWM_Value ? RF._flag_RC_Min_PWM_Value : EF._Tmp_B2_Speed;

            EF._Tmp_A1_Speed = EF._Tmp_A1_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_A1_Speed;
            EF._Tmp_A2_Speed = EF._Tmp_A2_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_A2_Speed;
            EF._Tmp_B1_Speed = EF._Tmp_B1_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_B1_Speed;
            EF._Tmp_B2_Speed = EF._Tmp_B2_Speed > RF._flag_RC_Max_PWM_Value ? RF._flag_RC_Max_PWM_Value : EF._Tmp_B2_Speed;

            EF._uORB_A1_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_A1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;
            EF._uORB_A2_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_A2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;
            EF._uORB_B1_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_B1_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;
            EF._uORB_B2_Speed = ((EF._Flag_Max__Throttle - EF._Flag_Lazy_Throttle) * (((float)EF._Tmp_B2_Speed - (float)RF._flag_RC_Min_PWM_Value) / (float)(RF._flag_RC_Max_PWM_Value - RF._flag_RC_Min_PWM_Value))) + EF._Flag_Lazy_Throttle;

            EF._uORB_A1_Speed = EF._uORB_A1_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_A1_Speed;
            EF._uORB_A2_Speed = EF._uORB_A2_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_A2_Speed;
            EF._uORB_B1_Speed = EF._uORB_B1_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_B1_Speed;
            EF._uORB_B2_Speed = EF._uORB_B2_Speed < EF._Flag_Lazy_Throttle ? EF._Flag_Lazy_Throttle : EF._uORB_B2_Speed;

            EF._uORB_A1_Speed = EF._uORB_A1_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_A1_Speed;
            EF._uORB_A2_Speed = EF._uORB_A2_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_A2_Speed;
            EF._uORB_B1_Speed = EF._uORB_B1_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_B1_Speed;
            EF._uORB_B2_Speed = EF._uORB_B2_Speed > EF._Flag_Max__Throttle ? EF._Flag_Max__Throttle : EF._uORB_B2_Speed;
            std::cout << "RF._uORB_A1_Speed " << EF._uORB_A1_Speed
                      << "\r\n";
            std::cout << "RF._uORB_A2_Speed " << EF._uORB_A2_Speed
                      << "\r\n";
            std::cout << "RF._uORB_B1_Speed " << EF._uORB_B1_Speed
                      << "\r\n";
            std::cout << "RF._uORB_B2_Speed " << EF._uORB_B2_Speed
                      << "\r\n";
        }
    }
    TF._Tmp_IMUAttThreadLast = GetTimestamp();
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
    // std::cout << "SF._uORB_Real__Roll" << SF._uORB_MPU_Data._uORB_Real__Roll
    //           << "\r\n";
    // std::cout << "SF._uORB_Real_Pitch" << SF._uORB_MPU_Data._uORB_Real_Pitch
    //           << "\r\n";
    // std::cout << "SF._uORB_Gryo__Roll" << SF._uORB_MPU_Data._uORB_Gryo__Roll
    //           << "\r\n";
    // std::cout << "SF._uORB_Gryo_Pitch" << SF._uORB_MPU_Data._uORB_Gryo_Pitch
    //           << "\r\n";
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
