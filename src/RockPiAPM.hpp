#pragma once
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <sys/time.h>
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
        int RockPiAPMInit(APMSettinngs APMInit);

        int APMCalibrator(int controller, int action, int input, double *data);

    protected:
        struct ESCINFO
        {
            int ESCPLFrequency = 1526;
            GeneratorType ESCControllerType = GeneratorType::Hardware_ONESHOT125;

            int _flag_A1_Pin = 0;
            int _flag_A2_Pin = 1;
            int _flag_B1_Pin = 2;
            int _flag_B2_Pin = 3;

            const int _Flag_Lock_Throttle = 1000;
            const int _Flag_Max__Throttle = 2000;

        } EF;
        struct TaskThread
        {
            int _flag_SystemStartUp_Time = 0;

        } TF;

        struct DeviceINFO
        {
            std::string I2CDevice;

            std::unique_ptr<ESCGenerator> ESCDevice;
        } DF;

    private:
        void ConfigReader(APMSettinngs APMInit);

        int GetTimestamp();
    };
}