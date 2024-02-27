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
#include "_thirdparty/RockPiMPU/src/_thirdparty/eigen/Eigen/Dense"
#include "_thirdparty/RockPiMPU/src/_thirdparty/eigen/Eigen/LU"

#define CaliESCUserDefine 2
#define ACCELCalibration 11
#define COMPASSCalibration 12
#define ESCCalibration 10
#define CaliESCStart 0
#define I2CPCA_ADDR 0x70

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

        } PC;

        struct SensorConfig
        {

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

        int RockPiAPMInit(APMSettinngs APMInit);

        int APMCalibrator(int controller, int action, int input, double *data);

    protected:
        void IMUSensorsTaskReg();

        void ControllerTaskReg();

        void ESCUpdateTaskReg();

        struct SafyINFO
        {
            bool _flag_ESC_ARMED = false;

        } AF;
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

        } EF;

        struct TaskThread
        {

            int _flag_SystemStartUp_Time = 0;
            const int _flag_Sys_CPU_Asign = 2;
            float _flag_IMUFlowFreq = 1000.f;
            float _flag_RTXFlowFreq = 250.f;
            float _flag_ESCFlowFreq = 1000.f;

            std::unique_ptr<FlowThread> IMUFlow;
            std::unique_ptr<FlowThread> RTXFlow;
            std::unique_ptr<FlowThread> ESCFlow;

        } TF;

        struct DeviceINFO
        {
            std::string I2CDevice;
            std::mutex I2CLock;

            std::unique_ptr<ESCGenerator> ESCDevice;
            std::unique_ptr<RPiMPU6500> MPUDevice;
        } DF;

        struct SensorsINFO
        {
            //=============MPU================//
            MPUData _uORB_MPU_Data;
            double _flag_MPU_Accel_Cali[20];

        } SF;

    private:
        void ConfigReader(APMSettinngs APMInit);

        int GetTimestamp();
    };
}