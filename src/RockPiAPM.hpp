#pragma once
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <sys/time.h>
#include <csignal>
#include "_thirdparty/ESCGenerator.hpp"
#include "_thirdparty/FlowController.hpp"
#include "_thirdparty/PCA9685/pca9685.hpp"
#include "_thirdparty/RockPiRC/src/M10QGPS/M10QGPS.hpp"
#include "_thirdparty/RockPiRC/src/QMC5883/QMC5883.hpp"
#include "_thirdparty/RaspberryPiBARO/src/BMP280/BMP280.hpp"
#include "_thirdparty/RockPiMPU/src/MPU6500/MPU6500.hpp"
#include "_thirdparty/RockPiRC/src/CRSF/CRSFUart.hpp"
#include "_thirdparty/RockPiMPU/src/_thirdparty/eigen/Eigen/Dense"
#include "_thirdparty/RockPiMPU/src/_thirdparty/eigen/Eigen/LU"

#define CaliESCUserDefine 2
#define ACCELCalibration 11
#define COMPASSCalibration 12
#define ESCCalibration 10
#define CaliESCStart 0
#define I2CPCA_ADDR 0x70

#define RCIsIbus 0
#define RCIsSbus 1
#define RCIsCRSF 2

#define FILTERIMUDTLPFCUTOFF 1.f
#define FILTERBAROLPFCUTOFF 1.f
#define FILTERTHROTTLELPFCUTOFF 4.f
#define FILTERPOSOUTLPFCUTOFF 4.f
#define FILTERMAGCUTOFF 5.f

#define PID_DT_DEFAULT 250.f

namespace RockPiAPMAPI
{
    inline volatile std::sig_atomic_t SystemSignal;

    struct APMSettinngs
    {
        struct DeviceConfig
        {
            std::string __I2CDevice;

        } DC;

        struct PIDConfig
        {
            float _flag_PID_Rate_Limit;

        } PC;

        struct SensorConfig
        {
            int _flag_MPU_Flip__Roll;
            int _flag_MPU_Flip_Pitch;
            int _flag_MPU_Flip___Yaw;
            double _flag_Accel__Roll_Cali;
            double _flag_Accel_Pitch_Cali;
            double _flag_MPU9250_A_X_Cali;
            double _flag_MPU9250_A_Y_Cali;
            double _flag_MPU9250_A_Z_Cali;
            double _flag_MPU9250_A_X_Scal;
            double _flag_MPU9250_A_Y_Scal;
            double _flag_MPU9250_A_Z_Scal;
        } SC;

        struct OutputConfig
        {
            int _flag_A1_Pin;
            int _flag_A2_Pin;
            int _flag_B1_Pin;
            int _flag_B2_Pin;

            int ESCPLFrequency;
            int ESCControllerType;
        } OC;

        struct RCConfig
        {

        } RC;
        struct FilterConfig
        {

        } FC;
    };
    class RockPiAPM
    {
    public:
        void RockPiAPMStartUp();

        void TaskThreadBlock();

        int RockPiAPMInit(APMSettinngs APMInit);

        int APMCalibrator(int controller, int action, int input, double *data);

        void RPiSingleAPMDeInit();

        ~RockPiAPM() { RPiSingleAPMDeInit(); }

    protected:
        void IMUSensorsTaskReg();

        void ControllerTaskReg();

        void ESCUpdateTaskReg();

        struct SafyINFO
        {
            bool _flag_ESC_ARMED = false;
            bool _flag_MPUCalibrating = false;

        } AF;

        struct PIDINFO
        {
            float _flag_PID_Rate_Limit = 500.f;

            float _uORB_PID_GYaw_Output;
            float _uORB_PID_AngleRate_Pitch;
            float _uORB_PID_AngleRate__Roll;

            float _uORB_PID_D_Last_Value__Roll = 0;
            float _uORB_PID_D_Last_Value_Pitch = 0;
            float _uORB_PID_D_Last_Value___Yaw = 0;

            float _uORB_PID_I_Last_Value__Roll = 0;
            float _uORB_PID_I_Last_Value_Pitch = 0;
            float _uORB_PID_I_Last_Value___Yaw = 0;

            float _uORB_PID__Roll_Input = 0;
            float _uORB_PID_Pitch_Input = 0;

            float _uORB_Leveling__Roll = 0;
            float _uORB_Leveling_Pitch = 0;
            float _uORB_Leveling___Yaw = 0;

            float _flag_PID_P__Roll_Gain = 0;
            float _flag_PID_P_Pitch_Gain = 0;
            float _flag_PID_P___Yaw_Gain = 0;

            float _flag_PID_I__Roll_Gain = 0;
            float _flag_PID_I_Pitch_Gain = 0;
            float _flag_PID_I___Yaw_Gain = 0;
            float _flag_PID_I__Roll_Max__Value = 0;
            float _flag_PID_I_Pitch_Max__Value = 0;
            float _flag_PID_I___Yaw_Max__Value = 0;
            float _uORB_PID_I_Dynamic_Gain = 1.f;

            float _flag_PID_D__Roll_Gain = 0;
            float _flag_PID_D_Pitch_Gain = 0;
            float _flag_PID_D___Yaw_Gain = 0;

			float _flag_PID_Level_Max = 0;

            float _uORB_PID_TPA_Beta = 1.f;

            float _flag_Filter_PID_I_CutOff = 30.f;
            float _flag_Filter_PID_D_ST1_CutOff = 100.f;
            float _flag_Filter_PID_D_ST2_CutOff = 200.f;
            float _flag_Filter_AngleRate_CutOff = 20;

			float _flag_PID_AngleRate___Yaw_Gain = 5.f;

        } PF;
        struct ESCINFO
        {
            int ESCPLFrequency = 1526;
            GeneratorType ESCControllerType = GeneratorType::Hardware_ONESHOT125;

            int _flag_A1_Pin = 0;
            int _flag_A2_Pin = 1;
            int _flag_B1_Pin = 2;
            int _flag_B2_Pin = 3;
            int _uORB_A1_Speed = 0;
            int _uORB_A2_Speed = 0;
            int _uORB_B1_Speed = 0;
            int _uORB_B2_Speed = 0;
            const int _Flag_Lock_Throttle = 1000;
            const int _Flag_Max__Throttle = 2000;

			float _flag_YAWOut_Reverse = 1.f;

        } EF;

        struct TaskThread
        {

            int _flag_SystemStartUp_Time = 0;
            const int _flag_Sys_CPU_Asign = 2;
            float _flag_IMUFlowFreq = 1000.f;
            float _flag_RTXFlowFreq = 250.f;
            float _flag_ESCFlowFreq = 1000.f;

            float _Tmp_IMUAttThreadDT = 0;
            float _Tmp_IMUAttThreadLast = 0;
            float _uORB_IMUAttThreadDT = 0;

            std::unique_ptr<FlowThread> IMUFlow;
            std::unique_ptr<FlowThread> RTXFlow;
            std::unique_ptr<FlowThread> ESCFlow;

            bool _flag_Block_Task_Running = false;

        } TF;

        struct DeviceINFO
        {
            std::string I2CDevice;
            std::string RCDevice;
            std::string MPUDeviceSPI;

            std::mutex I2CLock;
            std::unique_ptr<ESCGenerator> ESCDevice;
            std::unique_ptr<RPiMPU6500> MPUDevice;
            std::unique_ptr<CRSF> CRSFInit;

            pt1Filter_t AngleRateLPF[3];
            pt1Filter_t IMUDtLPF;
            pt1Filter_t ItermFilterRoll;
            pt1Filter_t DtermFilterRoll;
            pt1Filter_t DtermFilterRollST2;
			pt1Filter_t ItermFilterPitch;
			pt1Filter_t DtermFilterPitch;
			pt1Filter_t DtermFilterPitchST2;

        } DF;

        struct SensorsINFO
        {
            //=============MPU================//
            MPUData _uORB_MPU_Data;
            int _flag_MPU_Flip__Roll;
            int _flag_MPU_Flip_Pitch;
            int _flag_MPU_Flip___Yaw;
            double _flag_MPU_Accel_Cali[20];
            int _flag_Filter_GYaw_CutOff = 0;

        } SF;

        struct RCINFO
        {
            int RC_Type;
            int _uORB_RC_Channel_PWM[16] = {1500, 1500, 1500, 1500, 2000, 1000, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            int _Tmp_RC_Data[36] = {0};

			int _uORB_RC_Out__Roll = 0;
			int _uORB_RC_Out_Pitch = 0;
			int _uORB_RC_Out_Throttle = 0;
			int _uORB_RC_Out___Yaw = 0;
        } RF;

    private:
        void ConfigReader(APMSettinngs APMInit);

        void SaftyCheck();

        void DebugOutPut();

        int GetTimestamp();

        void AttitudeUpdate();

        void PID_Caculate(float inputData, float &outputData,
                          float &last_I_Data, float &last_D_Data,
                          float P_Gain, float I_Gain, float D_Gain, float I_Max);
        void PID_CaculateExtend(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                                float &last_I_Data, float &last_D_Data,
                                float P_Gain, float I_Gain, float D_Gain, float I_Max);

        void PID_CaculateHyper(float inputDataP, float inputDataI, float inputDataD, float &outputData,
                               float &last_I_Data, float &last_D_Data,
                               float P_Gain, float I_Gain, float D_Gain, float I_Max);
    };

}