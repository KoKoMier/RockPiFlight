#pragma once
#include <iostream>
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <sys/time.h>
#include "_thirdparty/PCA9685/pca9685.hpp"
#include "_thirdparty/RockPiRC/src/M10QGPS/M10QGPS.hpp"
#include "_thirdparty/RockPiRC/src/QMC5883/QMC5883.hpp"
#include "_thirdparty/RaspberryPiBARO/src/BMP280/BMP280.hpp"
#include "_thirdparty/RockPiMPU/src/MPU6500/MPU6500.hpp"
#include "_thirdparty/RockPiMPU/src/_thirdparty/eigen/Eigen/Dense"
#include "_thirdparty/RockPiMPU/src/_thirdparty/eigen/Eigen/LU"

#define ESCCalibration 10
#define CaliESCStart 0
#define I2CPCA_ADDR 0x70

namespace RockPiAPMAPI
{
    struct APMSettinngs
    {
        struct DeviceConfig
        {

        } DC;

        struct PIDConfig
        {

        } PC;

        struct SensorConfig
        {

        } SC;

        struct OutputConfig
        {

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
        struct TaskThread
        {
            int _flag_SystemStartUp_Time = 0;

        } TF;

    private:
        int GetTimestamp();
    };
}