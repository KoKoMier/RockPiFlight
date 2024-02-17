#pragma once
#include <iostream>
#include "../SPI/LinuxSPI.hpp"
#include "MadgwickAHRS.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <time.h>
#include "filter.h"
#include "../_thirdparty/eigen/Eigen/Dense"
#include "../_thirdparty/eigen/Eigen/LU"

#define MPU6500_ACCEL_LSB 2048.f
#define MAX_ACC_NEARNESS 0.33 // 33% or G error soft-accepted (0.67-1.33G)
#define GravityAccel 9.80665

#define MPUTypeI2C 0
#define MPUTypeSPI 1
#define MPUMixKalman 1
#define MPUMixTradition 0
#define MPUAccelNoseUp 0
#define MPUAccelNoseDown 1
#define MPUAccelNoseRight 2
#define MPUAccelNoseLeft 3
#define MPUAccelNoseTop 4
#define MPUAccelNoseRev 5
#define MPUAccelCaliGet 6
#define MPUAccelCaliX 7
#define MPUAccelCaliY 8
#define MPUAccelCaliZ 9
#define MPUAccelScalX 10
#define MPUAccelScalY 11
#define MPUAccelScalZ 12
#define MPUAccelTRIM_Roll 13
#define MPUAccelTRIMPitch 14

#define ACC_CLIPPING_THRESHOLD_G 7.9f
#define DYNAMIC_NOTCH_DEFAULT_CENTER_HZ 350.f
#define DYN_NOTCH_SMOOTH_FREQ_HZ 50.f
#define ACC_VIBE_FLOOR_FILT_HZ 5.f
#define ACC_VIBE_FILT_HZ 2.f

#define FilterLPFPT1 0
#define FilterLPFBiquad 1
#define MPUTypeSPI 1
#define PI 3.1415926
#define MPU_250HZ_LPF_SPEED 8000.f
#define DEG2RAD(x) (x * PI / 180.f)

#define MPUTypeI2C 0
#define MPUTypeSPI 1

#define GAXR 0
#define GAYP 1
#define GAZY 2
#define AAXN 0
#define AAYN 1
#define AAZN 2

#define IMU_CALI_MAX_LOOP 500.0

#define DEG2RAD(x) (x * PI / 180.f)

struct MPUConfig
{
    bool MPUType = MPUTypeSPI;
    const char *MPUSPIChannel = "/dev/spidev0.0";
    uint8_t MPUI2CAddress = 0x68;
    float MPU_Flip_Pitch = 0;
    float MPU_Flip__Roll = 0;
    float MPU_Flip___Yaw = 0;

    int TargetFreqency = 1000;

    int MPU6500_SPI_Freq = 400000;
    float MPU_6500_LSB = 65.5 / 4;

    bool DynamicNotchEnable = true;

    float GyroToAccelBeta = 0.02;

    int GyroFilterNotchCutOff = 0;
    int GyroFilterType = FilterLPFPT1;
    int GyroFilterCutOff = 90;
    int GyroFilterTypeST2 = FilterLPFPT1;
    int GyroFilterCutOffST2 = 0;

    int AccelFilterType = FilterLPFBiquad;
    int AccTargetFreqency = 1000;
    int AccelFilterCutOff = 30;

    int AccelFilterNotchCutOff = 0;
};

struct MPUData
{
    int DeviceType = 0;

    int _uORB_MPU6500_A_X = 0;
    int _uORB_MPU6500_A_Y = 0;
    int _uORB_MPU6500_A_Z = 0;
    int _uORB_MPU6500_G_X = 0;
    int _uORB_MPU6500_G_Y = 0;
    int _uORB_MPU6500_G_Z = 0;
    int _uORB_MPU6500_M_X = 0;
    int _uORB_MPU6500_M_Y = 0;
    int _uORB_MPU6500_M_Z = 0;

    float _uORB_MPU6500_ADF_X = 0;
    float _uORB_MPU6500_ADF_Y = 0;
    float _uORB_MPU6500_ADF_Z = 0;

    float _uORB_MPU6500_A_Vector = 0;
    float _uORB_MPU6500_A_Static_Vector = 0;

    float _uORB_Accel_VIBE_X = 0;
    float _uORB_Accel_VIBE_Y = 0;
    float _uORB_Accel_VIBE_Z = 0;

    float _uORB_Gryo__Roll = 0;
    float _uORB_Gryo_Pitch = 0;
    float _uORB_Gryo___Yaw = 0;
    float _uORB_Real__Roll = 0;
    float _uORB_Real_Pitch = 0;
    float _uORB_Real___Yaw = 0;
    float _uORB_Accel__Roll = 0;
    float _uORB_Accel_Pitch = 0;
    float _uORB_Acceleration_X = 0;
    float _uORB_Acceleration_Y = 0;
    float _uORB_Acceleration_Z = 0;
    float _uORB_Raw_QuaternionQ[4] = {0};

    double _flag_MPU6500_A_X_Scal = 1.f;
    double _flag_MPU6500_A_Y_Scal = 1.f;
    double _flag_MPU6500_A_Z_Scal = 1.f;
    double _flag_MPU6500_A_X_Cali = 0;
    double _flag_MPU6500_A_Y_Cali = 0;
    double _flag_MPU6500_A_Z_Cali = 0;
    double _flag_MPU6500_A_TP_Cali = 0;
    double _flag_MPU6500_A_TR_Cali = 0;

    float _uORB_MPU6500_A_Static_X = 0;
    float _uORB_MPU6500_A_Static_Y = 0;
    float _uORB_MPU6500_A_Static_Z = 0;

    int _flag_MPU6500_G_X_Cali;
    int _flag_MPU6500_G_Y_Cali;
    int _flag_MPU6500_G_Z_Cali;

    bool _uORB_MPU6500_ACC_Clipped = false;
    float MPUMixTraditionBeta = 0.05;

    Eigen::Matrix3d _uORB_MPU6500_RotationMatrix;
    Eigen::Quaternion<double> _uORB_MPU6500_Quaternion;

    int _uORB_MPU6500_IMUUpdateTime = 0;
    int _uORB_MPU6500_AccelCountDown = 0;
};

class RPiMPU6500
{
public:
    inline RPiMPU6500()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        IMUstartuptime = (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))));
        LastUpdate = GetTimestamp();

        PrivateData._uORB_MPU6500_IMUUpdateTime = GetTimestamp();
        // GyroDynmiacNotchMinBox = (PrivateConfig.DynamicNotchMinFreq / FFTResolution) - 1;
        {
            int DT = (float)(1.f / (float)PrivateConfig.TargetFreqency) * 1000000;
            int ACCDT = (float)(1.f / (float)PrivateConfig.AccTargetFreqency) * 1000000;
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
            {
                pt1FilterInit(&GryoFilterLPF[GAXR], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPF[GAYP], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPF[GAZY], PrivateConfig.GyroFilterCutOff, DT * 1e-6f);
            }
            break;
            case FilterLPFBiquad:
            {
                biquadFilterInitLPF(&GryoFilterBLPF[GAXR], PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPF[GAYP], PrivateConfig.GyroFilterCutOff, DT);
                biquadFilterInitLPF(&GryoFilterBLPF[GAZY], PrivateConfig.GyroFilterCutOff, DT);
            }
            break;
            }
            switch (PrivateConfig.GyroFilterTypeST2)
            {
            case FilterLPFPT1:
            {
                pt1FilterInit(&GryoFilterLPFST2[GAXR], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFST2[GAYP], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
                pt1FilterInit(&GryoFilterLPFST2[GAZY], PrivateConfig.GyroFilterCutOffST2, DT * 1e-6f);
            }
            break;
            case FilterLPFBiquad:
            {
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAXR], PrivateConfig.GyroFilterCutOffST2, DT);
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAYP], PrivateConfig.GyroFilterCutOffST2, DT);
                biquadFilterInitLPF(&GryoFilterBLPFST2[GAZY], PrivateConfig.GyroFilterCutOffST2, DT);
            }
            break;
            }
            switch (PrivateConfig.AccelFilterType)
            {
            case FilterLPFPT1:
            {
                pt1FilterInit(&AccelFilterLPF[AAXN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                pt1FilterInit(&AccelFilterLPF[AAYN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
                pt1FilterInit(&AccelFilterLPF[AAZN], PrivateConfig.AccelFilterCutOff, ACCDT * 1e-6f);
            }
            break;
            case FilterLPFBiquad:
            {
                biquadFilterInitLPF(&AccelFilterBLPF[AAXN], PrivateConfig.AccelFilterCutOff, ACCDT);
                biquadFilterInitLPF(&AccelFilterBLPF[AAYN], PrivateConfig.AccelFilterCutOff, ACCDT);
                biquadFilterInitLPF(&AccelFilterBLPF[AAZN], PrivateConfig.AccelFilterCutOff, ACCDT);
            }
            break;
            }
            if (PrivateConfig.DynamicNotchEnable)
            {
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAXR], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAYP], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInitLPF(&GryoFilterDynamicFreq[GAZY], DYN_NOTCH_SMOOTH_FREQ_HZ, (DT * 16 * (PrivateConfig.TargetFreqency / 1000)));
                biquadFilterInit(&GryoFilterDynamicNotch[GAXR], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotch[GAYP], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
                biquadFilterInit(&GryoFilterDynamicNotch[GAZY], DYNAMIC_NOTCH_DEFAULT_CENTER_HZ, DT, 1.0f, FILTER_NOTCH);
            }
            pt1FilterInit(&VibeFloorLPF[AAXN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeFloorLPF[AAYN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeFloorLPF[AAZN], ACC_VIBE_FLOOR_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAXN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAYN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
            pt1FilterInit(&VibeLPF[AAZN], ACC_VIBE_FILT_HZ, ACCDT * 1e-6f);
        }
        IMUSensorsDeviceInit();
        AHRSSys.reset(new MadgwickAHRS(PrivateConfig.GyroToAccelBeta, PrivateConfig.TargetFreqency));
    }

    inline int MPUCalibration(double *AccelCaliData)
    {
        PrivateData._flag_MPU6500_A_X_Scal = AccelCaliData[MPUAccelScalX];
        PrivateData._flag_MPU6500_A_Y_Scal = AccelCaliData[MPUAccelScalY];
        PrivateData._flag_MPU6500_A_Z_Scal = AccelCaliData[MPUAccelScalZ];
        PrivateData._flag_MPU6500_A_X_Cali = AccelCaliData[MPUAccelCaliX];
        PrivateData._flag_MPU6500_A_Y_Cali = AccelCaliData[MPUAccelCaliY];
        PrivateData._flag_MPU6500_A_Z_Cali = AccelCaliData[MPUAccelCaliZ];
        PrivateData._flag_MPU6500_A_TR_Cali = AccelCaliData[MPUAccelTRIM_Roll];
        PrivateData._flag_MPU6500_A_TP_Cali = AccelCaliData[MPUAccelTRIMPitch];

        double _Tmp_Gryo_X_Cali = 0;
        double _Tmp_Gryo_Y_Cali = 0;
        double _Tmp_Gryo_Z_Cali = 0;
        double _Tmp_Accel_Static_Cali = 0;
        PrivateData._flag_MPU6500_G_X_Cali = 0;
        PrivateData._flag_MPU6500_G_Y_Cali = 0;
        PrivateData._flag_MPU6500_G_Z_Cali = 0;

        for (int cali_count = 0; cali_count < IMU_CALI_MAX_LOOP; cali_count++)
        {
            MPUSensorsDataGet();
            ResetMPUMixAngle();
            _Tmp_Gryo_X_Cali += PrivateData._uORB_MPU6500_G_X;
            _Tmp_Gryo_Y_Cali += PrivateData._uORB_MPU6500_G_Y;
            _Tmp_Gryo_Z_Cali += PrivateData._uORB_MPU6500_G_Z;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }
        for (int cali_count = 0; cali_count < IMU_CALI_MAX_LOOP; cali_count++)
        {
            MPUSensorsDataGet();
            _Tmp_Accel_Static_Cali += PrivateData._uORB_MPU6500_A_Vector;
            usleep((int)(1.f / (float)PrivateConfig.TargetFreqency * 1000000.f));
        }
        PrivateData._flag_MPU6500_G_X_Cali = _Tmp_Gryo_X_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_MPU6500_G_Y_Cali = _Tmp_Gryo_Y_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._flag_MPU6500_G_Z_Cali = _Tmp_Gryo_Z_Cali / IMU_CALI_MAX_LOOP;
        PrivateData._uORB_MPU6500_A_Static_Vector = _Tmp_Accel_Static_Cali / IMU_CALI_MAX_LOOP;
        return 0;
    }

    inline void MPUAccelCalibration(int AccelCaliAction, double *AccelCaliData)
    {
        int AccelCaliTmpTotal = 0;
        AccelCaliData[AccelCaliAction] = 0;
        for (int cali_count = 0; cali_count < 2000; cali_count++)
        {
            IMUSensorsDataRead();
            switch (AccelCaliAction)
            {
            case MPUAccelNoseUp:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU6500_A_X;
                break;
            case MPUAccelNoseDown:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU6500_A_X;
                break;
            case MPUAccelNoseRight:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU6500_A_Y;
                break;
            case MPUAccelNoseLeft:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU6500_A_Y;
                break;
            case MPUAccelNoseTop:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU6500_A_Z;
                break;
            case MPUAccelNoseRev:
                AccelCaliData[AccelCaliAction] += PrivateData._uORB_MPU6500_A_Z;
                break;
            }
            usleep((int)(1.f / (float)PrivateConfig.AccTargetFreqency * 1000000.f));
        }
        if (AccelCaliAction == MPUAccelCaliGet)
        {
            AccelCaliData[MPUAccelNoseUp] /= 2000.f;
            AccelCaliData[MPUAccelNoseDown] /= 2000.f;
            AccelCaliData[MPUAccelNoseRight] /= 2000.f;
            AccelCaliData[MPUAccelNoseLeft] /= 2000.f;
            AccelCaliData[MPUAccelNoseTop] /= 2000.f;
            AccelCaliData[MPUAccelNoseRev] /= 2000.f;
            std::cout << "data-x-max:" << AccelCaliData[MPUAccelNoseUp] << "\r\n";
            std::cout << "data-x-min:" << AccelCaliData[MPUAccelNoseDown] << "\r\n";
            std::cout << "data-y-max:" << AccelCaliData[MPUAccelNoseRight] << "\r\n";
            std::cout << "data-y-min:" << AccelCaliData[MPUAccelNoseLeft] << "\r\n";
            std::cout << "data-z-max:" << AccelCaliData[MPUAccelNoseTop] << "\r\n";
            std::cout << "data-z-min:" << AccelCaliData[MPUAccelNoseRev] << "\r\n";

            AccelCaliData[MPUAccelScalX] = std::abs(MPU6500_Accel_LSB / ((AccelCaliData[MPUAccelNoseUp] - AccelCaliData[MPUAccelNoseDown]) / 2.f));
            AccelCaliData[MPUAccelScalY] = std::abs(MPU6500_Accel_LSB / ((AccelCaliData[MPUAccelNoseLeft] - AccelCaliData[MPUAccelNoseRight]) / 2.f));
            AccelCaliData[MPUAccelScalZ] = std::abs(MPU6500_Accel_LSB / ((AccelCaliData[MPUAccelNoseTop] - AccelCaliData[MPUAccelNoseRev]) / 2.f));
            AccelCaliData[MPUAccelCaliX] = ((AccelCaliData[MPUAccelNoseUp] + AccelCaliData[MPUAccelNoseDown]) / 2.f) * AccelCaliData[MPUAccelScalX];
            AccelCaliData[MPUAccelCaliY] = ((AccelCaliData[MPUAccelNoseRight] + AccelCaliData[MPUAccelNoseLeft]) / 2.f) * AccelCaliData[MPUAccelScalY];
            AccelCaliData[MPUAccelCaliZ] = ((AccelCaliData[MPUAccelNoseTop] + AccelCaliData[MPUAccelNoseRev]) / 2.f) * AccelCaliData[MPUAccelScalZ];
        }
    }

    inline MPUData MPUSensorsDataGet()
    {
        IMUSensorsDataRead();

        PrivateData._uORB_MPU6500_G_X -= PrivateData._flag_MPU6500_G_X_Cali;
        PrivateData._uORB_MPU6500_G_Y -= PrivateData._flag_MPU6500_G_Y_Cali;
        PrivateData._uORB_MPU6500_G_Z -= PrivateData._flag_MPU6500_G_Z_Cali;

        PrivateData._uORB_MPU6500_A_X = PrivateData._uORB_MPU6500_A_X * PrivateData._flag_MPU6500_A_X_Scal - PrivateData._flag_MPU6500_A_X_Cali;
        PrivateData._uORB_MPU6500_A_Y = PrivateData._uORB_MPU6500_A_Y * PrivateData._flag_MPU6500_A_Y_Scal - PrivateData._flag_MPU6500_A_Y_Cali;
        PrivateData._uORB_MPU6500_A_Z = PrivateData._uORB_MPU6500_A_Z * PrivateData._flag_MPU6500_A_Z_Scal - PrivateData._flag_MPU6500_A_Z_Cali;

        //========================= //=========================Gyro Filter
        {
            switch (PrivateConfig.GyroFilterType)
            {
            case FilterLPFPT1:
            {
                if (PrivateConfig.GyroFilterCutOff)
                {
                    PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPF[GAXR], ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB));
                    PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPF[GAYP], ((float)PrivateData._uORB_MPU6500_G_Y / PrivateConfig.MPU_6500_LSB));
                    PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPF[GAZY], ((float)PrivateData._uORB_MPU6500_G_Z / PrivateConfig.MPU_6500_LSB));
                }
                else
                {
                    PrivateData._uORB_Gryo__Roll = ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB);
                    PrivateData._uORB_Gryo_Pitch = ((float)PrivateData._uORB_MPU6500_G_Y / PrivateConfig.MPU_6500_LSB);
                    PrivateData._uORB_Gryo___Yaw = ((float)PrivateData._uORB_MPU6500_G_Z / PrivateConfig.MPU_6500_LSB);
                }
                break;
            }
            case FilterLPFBiquad:
            {
                if (PrivateConfig.GyroFilterCutOff)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPF[GAXR], ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB));
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPF[GAYP], ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB));
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPF[GAZY], ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB));
                }
                else
                {
                    PrivateData._uORB_Gryo__Roll = ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB);
                    PrivateData._uORB_Gryo_Pitch = ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB);
                    PrivateData._uORB_Gryo___Yaw = ((float)PrivateData._uORB_MPU6500_G_X / PrivateConfig.MPU_6500_LSB);
                }
            }
            break;
            }
            switch (PrivateConfig.GyroFilterTypeST2)
            {
            case FilterLPFPT1:
                if (PrivateConfig.GyroFilterCutOffST2)
                {
                    PrivateData._uORB_Gryo__Roll = pt1FilterApply(&GryoFilterLPFST2[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = pt1FilterApply(&GryoFilterLPFST2[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = pt1FilterApply(&GryoFilterLPFST2[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
                break;
            case FilterLPFBiquad:
                if (PrivateConfig.GyroFilterCutOffST2)
                {
                    PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GryoFilterBLPFST2[GAXR], PrivateData._uORB_Gryo__Roll);
                    PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GryoFilterBLPFST2[GAYP], PrivateData._uORB_Gryo_Pitch);
                    PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GryoFilterBLPFST2[GAZY], PrivateData._uORB_Gryo___Yaw);
                }
                break;
            }

            if (PrivateConfig.GyroFilterNotchCutOff)
            {
                PrivateData._uORB_Gryo_Pitch = biquadFilterApply(&GyroNotchFilter[GAXR], PrivateData._uORB_Gryo_Pitch);
                PrivateData._uORB_Gryo__Roll = biquadFilterApply(&GyroNotchFilter[GAYP], PrivateData._uORB_Gryo__Roll);
                PrivateData._uORB_Gryo___Yaw = biquadFilterApply(&GyroNotchFilter[GAZY], PrivateData._uORB_Gryo___Yaw);
            }
        }
        //========================= //=========================Dynamic Anlyisc
        {
            if (PrivateConfig.TargetFreqency >= 1000)
            {
            }
        }
        //========================= //=========================Accel Filter
        {
            if (PrivateData._uORB_MPU6500_AccelCountDown == 1)
            {
                float AccVibeFloorX = pt1FilterApply(&VibeFloorLPF[AAXN], (PrivateData._uORB_MPU6500_A_X / PrivateConfig.MPU_6500_LSB));
                float AccVibeFloorY = pt1FilterApply(&VibeFloorLPF[AAYN], (PrivateData._uORB_MPU6500_A_Y / PrivateConfig.MPU_6500_LSB));
                float AccVibeFloorZ = pt1FilterApply(&VibeFloorLPF[AAZN], (PrivateData._uORB_MPU6500_A_Z / PrivateConfig.MPU_6500_LSB));
                PrivateData._uORB_Accel_VIBE_X = pt1FilterApply(&VibeLPF[AAXN], (pow((((float)PrivateData._uORB_MPU6500_A_X / PrivateConfig.MPU_6500_LSB) - AccVibeFloorX), 2)));
                PrivateData._uORB_Accel_VIBE_Y = pt1FilterApply(&VibeLPF[AAYN], (pow((((float)PrivateData._uORB_MPU6500_A_Y / PrivateConfig.MPU_6500_LSB) - AccVibeFloorY), 2)));
                PrivateData._uORB_Accel_VIBE_Z = pt1FilterApply(&VibeLPF[AAZN], (pow((((float)PrivateData._uORB_MPU6500_A_Z / PrivateConfig.MPU_6500_LSB) - AccVibeFloorZ), 2)));

                //
                if (PrivateConfig.AccelFilterCutOff)
                {
                    switch (PrivateConfig.AccelFilterType)
                    {
                    case FilterLPFPT1:
                        PrivateData._uORB_MPU6500_ADF_X = pt1FilterApply(&AccelFilterLPF[AAXN], ((float)PrivateData._uORB_MPU6500_A_X / PrivateConfig.MPU_6500_LSB));
                        PrivateData._uORB_MPU6500_ADF_Y = pt1FilterApply(&AccelFilterLPF[AAYN], ((float)PrivateData._uORB_MPU6500_A_Y / PrivateConfig.MPU_6500_LSB));
                        PrivateData._uORB_MPU6500_ADF_Z = pt1FilterApply(&AccelFilterLPF[AAZN], ((float)PrivateData._uORB_MPU6500_A_Z / PrivateConfig.MPU_6500_LSB));

                        break;
                    case FilterLPFBiquad:
                        PrivateData._uORB_MPU6500_ADF_X = biquadFilterApply(&AccelFilterBLPF[AAXN], ((float)PrivateData._uORB_MPU6500_A_X / PrivateConfig.MPU_6500_LSB));
                        PrivateData._uORB_MPU6500_ADF_Y = biquadFilterApply(&AccelFilterBLPF[AAYN], ((float)PrivateData._uORB_MPU6500_A_Y / PrivateConfig.MPU_6500_LSB));
                        PrivateData._uORB_MPU6500_ADF_Z = biquadFilterApply(&AccelFilterBLPF[AAZN], ((float)PrivateData._uORB_MPU6500_A_Z / PrivateConfig.MPU_6500_LSB));

                        break;
                    }
                    if (PrivateConfig.AccelFilterNotchCutOff)
                    {
                        PrivateData._uORB_MPU6500_ADF_X = biquadFilterApply(&AccelNotchFilter[AAXN], PrivateData._uORB_MPU6500_ADF_X);
                        PrivateData._uORB_MPU6500_ADF_Y = biquadFilterApply(&AccelNotchFilter[AAYN], PrivateData._uORB_MPU6500_ADF_Y);
                        PrivateData._uORB_MPU6500_ADF_Z = biquadFilterApply(&AccelNotchFilter[AAZN], PrivateData._uORB_MPU6500_ADF_Z);
                    }
                }
                else
                {
                    PrivateData._uORB_MPU6500_ADF_X = PrivateData._uORB_MPU6500_ADF_X / PrivateConfig.MPU_6500_LSB;
                    PrivateData._uORB_MPU6500_ADF_Y = PrivateData._uORB_MPU6500_ADF_X / PrivateConfig.MPU_6500_LSB;
                    PrivateData._uORB_MPU6500_ADF_Z = PrivateData._uORB_MPU6500_ADF_X / PrivateConfig.MPU_6500_LSB;
                }

                //
                if (abs(PrivateData._uORB_MPU6500_ADF_X) > ACC_CLIPPING_THRESHOLD_G ||
                    abs(PrivateData._uORB_MPU6500_ADF_Y) > ACC_CLIPPING_THRESHOLD_G ||
                    abs(PrivateData._uORB_MPU6500_ADF_Z) > ACC_CLIPPING_THRESHOLD_G)
                    PrivateData._uORB_MPU6500_ACC_Clipped = true;
                else
                    PrivateData._uORB_MPU6500_ACC_Clipped = false;
                //
                PrivateData._uORB_MPU6500_A_Vector = sqrtf((PrivateData._uORB_MPU6500_ADF_X * PrivateData._uORB_MPU6500_ADF_X) +
                                                           (PrivateData._uORB_MPU6500_ADF_Y * PrivateData._uORB_MPU6500_ADF_Y) +
                                                           (PrivateData._uORB_MPU6500_ADF_Z * PrivateData._uORB_MPU6500_ADF_Z));
                PrivateData._uORB_Accel_Pitch = -1 * atan2((float)PrivateData._uORB_MPU6500_ADF_X, PrivateData._uORB_MPU6500_ADF_Z) * 180.f / PI;
                PrivateData._uORB_Accel__Roll = atan2((float)PrivateData._uORB_MPU6500_ADF_Y, PrivateData._uORB_MPU6500_ADF_Z) * 180.f / PI;

                //========================= //=========================AHRS Update
                {
                    if ((PrivateData._uORB_MPU6500_A_Vector > (PrivateData._uORB_MPU6500_A_Static_Vector - MAX_ACC_NEARNESS)) &&
                        (PrivateData._uORB_MPU6500_A_Vector < (PrivateData._uORB_MPU6500_A_Static_Vector + MAX_ACC_NEARNESS)))
                        PrivateData.MPUMixTraditionBeta = PrivateConfig.GyroToAccelBeta;
                    else
                        PrivateData.MPUMixTraditionBeta = 0.f;

                    if (!AHRSEnable)
                        AHRSSys->MadgwickAHRSIMUApply(PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo___Yaw,
                                                      (PrivateData._uORB_MPU6500_ADF_X),
                                                      (PrivateData._uORB_MPU6500_ADF_Y),
                                                      (PrivateData._uORB_MPU6500_ADF_Z),
                                                      ((float)PrivateData._uORB_MPU6500_IMUUpdateTime * 1e-6f));
                    else
                        AHRSSys->MadgwickAHRSApply(PrivateData._uORB_Gryo__Roll, PrivateData._uORB_Gryo_Pitch, PrivateData._uORB_Gryo___Yaw,
                                                   (PrivateData._uORB_MPU6500_ADF_X),
                                                   (PrivateData._uORB_MPU6500_ADF_Y),
                                                   (PrivateData._uORB_MPU6500_ADF_Z),
                                                   (_Tmp_AHRS_MAG_X),
                                                   (_Tmp_AHRS_MAG_Y),
                                                   (_Tmp_AHRS_MAG_Z),
                                                   ((float)PrivateData._uORB_MPU6500_IMUUpdateTime * 1e-6f));

                    AHRSSys->MadgwickSetAccelWeight(PrivateData.MPUMixTraditionBeta);
                    AHRSSys->MadgwickAHRSGetQ(PrivateData._uORB_Raw_QuaternionQ[0],
                                              PrivateData._uORB_Raw_QuaternionQ[1],
                                              PrivateData._uORB_Raw_QuaternionQ[2],
                                              PrivateData._uORB_Raw_QuaternionQ[3]);
                    AHRSSys->MadgwickComputeAngles(PrivateData._uORB_Real__Roll, PrivateData._uORB_Real_Pitch, PrivateData._uORB_Real___Yaw);

                    PrivateData._uORB_Real__Roll *= 180.f / PI;
                    PrivateData._uORB_Real_Pitch *= 180.f / PI;
                    PrivateData._uORB_Real___Yaw *= 180.f / PI;
                    PrivateData._uORB_Real___Yaw = PrivateData._uORB_Real___Yaw > 0 ? 360 - PrivateData._uORB_Real___Yaw : PrivateData._uORB_Real___Yaw;
                    PrivateData._uORB_Real___Yaw = PrivateData._uORB_Real___Yaw < 0 ? -1 * PrivateData._uORB_Real___Yaw : PrivateData._uORB_Real___Yaw;
                    //
                    PrivateData._uORB_Real__Roll += PrivateData._flag_MPU6500_A_TR_Cali;
                    PrivateData._uORB_Real_Pitch += PrivateData._flag_MPU6500_A_TP_Cali;
                    //========================= //=========================Navigation update
                    {
                        if (PrivateData._uORB_MPU6500_AccelCountDown == 1)
                        {
                            PrivateData._uORB_MPU6500_Quaternion = Eigen::AngleAxisd(((PrivateData._uORB_Real__Roll - PrivateData._flag_MPU6500_A_TR_Cali) * (PI / 180.f)), Eigen::Vector3d::UnitZ()) *
                                                                   Eigen::AngleAxisd(((PrivateData._uORB_Real_Pitch - PrivateData._flag_MPU6500_A_TP_Cali) * (PI / 180.f)), Eigen::Vector3d::UnitY()) *
                                                                   Eigen::AngleAxisd((0 * (PI / 180.f)), Eigen::Vector3d::UnitX());

                            PrivateData._uORB_MPU6500_RotationMatrix = PrivateData._uORB_MPU6500_Quaternion.normalized().toRotationMatrix();
                            Eigen::Matrix<double, 1, 3> AccelRaw;
                            AccelRaw << PrivateData._uORB_MPU6500_ADF_Z,
                                PrivateData._uORB_MPU6500_ADF_Y,
                                PrivateData._uORB_MPU6500_ADF_X;

                            Eigen::Matrix<double, 1, 3> AccelStatic = AccelRaw * PrivateData._uORB_MPU6500_RotationMatrix;
                            PrivateData._uORB_MPU6500_A_Static_Z = AccelStatic[0] - PrivateData._uORB_MPU6500_A_Static_Vector;
                            PrivateData._uORB_MPU6500_A_Static_X = -1.f * AccelStatic[1];
                            PrivateData._uORB_MPU6500_A_Static_Y = -1.f * AccelStatic[2];
                            PrivateData._uORB_Acceleration_X = ((float)PrivateData._uORB_MPU6500_A_Static_X) * GravityAccel * 100.f;
                            PrivateData._uORB_Acceleration_Y = ((float)PrivateData._uORB_MPU6500_A_Static_Y) * GravityAccel * 100.f;
                            PrivateData._uORB_Acceleration_Z = ((float)PrivateData._uORB_MPU6500_A_Static_Z) * GravityAccel * 100.f;
                        }
                    }
                }
            }
        }
        return PrivateData;
    }

    inline void ResetMPUMixAngle()
    {
        AHRSSys->MadgwickResetToAccel();
    }

    inline ~RPiMPU6500()
    {
        AHRSSys.reset();
        _s_spiClose(MPU6500_fd);
    };

private:
    inline void IMUSensorsDeviceInit()
    {
        // double OutputSpeedCal = (MPU_250HZ_LPF_SPEED / (float)PrivateConfig.TargetFreqency) - 1.f;
        if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            MPU6500_fd = _s_spiOpen(PrivateConfig.MPUSPIChannel, PrivateConfig.MPU6500_SPI_Freq, 0);
            if (MPU6500_fd < 0)
                throw std::invalid_argument("[SPI] MPU device can't open");
            uint8_t MPU6500_SPI_Config_WHOAMI[2] = {0xf5, 0x00};
            _s_spiXfer(MPU6500_fd, MPU6500_SPI_Config_WHOAMI, MPU6500_SPI_Config_WHOAMI, PrivateConfig.MPU6500_SPI_Freq, 2);
            PrivateData.DeviceType = MPU6500_SPI_Config_WHOAMI[1];

            uint8_t MPU6500_SPI_Config_RESET[2] = {0x6b, 0x80};
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_RESET, PrivateConfig.MPU6500_SPI_Freq, 2); // reset
            usleep(500);
            uint8_t MPU6500_SPI_Config_RESET2[2] = {0x68, 0x07};
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_RESET, PrivateConfig.MPU6500_SPI_Freq, 2); // BIT_GYRO | BIT_ACC | BIT_TEMP reset
            usleep(500);
            uint8_t MPU6500_SPI_Config_RESET3[2] = {0x6b, 0x00};
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_RESET3, PrivateConfig.MPU6500_SPI_Freq, 2); // reset
            usleep(500);
            uint8_t MPU6500_SPI_Config_RESET4[2] = {0x6b, 0x01};
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_RESET4, PrivateConfig.MPU6500_SPI_Freq, 2); // reset
            usleep(1000);

            uint8_t MPU6500_SPI_Config_ALPF[2] = {0x1d, 0x00};                                   // FChoice 1, DLPF 3 , dlpf cut off 44.8hz for accel is 0x03, but now 0x00 is not apply accel hardware
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_ALPF, PrivateConfig.MPU6500_SPI_Freq, 2); // Accel2
            usleep(15);
            uint8_t MPU6500_SPI_Config_Acce[2] = {0x1c, 0x18};                                   // Full AccelScale +- 16g
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_Acce, PrivateConfig.MPU6500_SPI_Freq, 2); // Accel
            usleep(15);
            uint8_t MPU6500_SPI_Config_Gyro[2] = {0x1b, 0x18};                                   // Full GyroScale +-2000dps, dlpf 250hz
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_Gyro, PrivateConfig.MPU6500_SPI_Freq, 2); // Gryo
            usleep(15);
            uint8_t MPU6500_SPI_Config_GLPF[2] = {0x1a, 0x00};                                   // DLPF_CFG is 000 , with Gyro dlpf is 250hz
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_GLPF, PrivateConfig.MPU6500_SPI_Freq, 2); // config
            usleep(15);

            uint8_t MPU6500_SPI_Config_INTC[2] = {0x37, 0x22};
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_INTC, PrivateConfig.MPU6500_SPI_Freq, 2);
            usleep(500);
            uint8_t MPU6500_SPI_Config_INTE[2] = {0x38, 0x01};
            _s_spiWrite(MPU6500_fd, MPU6500_SPI_Config_INTE, PrivateConfig.MPU6500_SPI_Freq, 2);
            usleep(500);
        }
        else if (PrivateConfig.MPUType == MPUTypeI2C)
        {
        }
    }

    inline void IMUSensorsDataRead()
    {
        if (PrivateConfig.MPUType == MPUTypeI2C)
        {
            //
        }
        else if (PrivateConfig.MPUType == MPUTypeSPI)
        {
            uint8_t Tmp_MPU6500_SPI_BufferX[2] = {0};
            Tmp_MPU6500_SPI_BufferX[0] = 0xBA;
            _s_spiXfer(MPU6500_fd, Tmp_MPU6500_SPI_BufferX, Tmp_MPU6500_SPI_BufferX, PrivateConfig.MPU6500_SPI_Freq, 2);

            if (Tmp_MPU6500_SPI_BufferX[1] & 0x01)
            {
                PrivateData._uORB_MPU6500_IMUUpdateTime = GetTimestamp() - LastUpdate;
                LastUpdate = GetTimestamp();

                uint8_t Tmp_MPU6500_SPI_Buffer[8] = {0};
                uint8_t Tmp_MPU6500_SPI_Bufferout[8] = {0};
                Tmp_MPU6500_SPI_Buffer[0] = 0xBB;
                if (PrivateData._uORB_MPU6500_AccelCountDown >= (PrivateConfig.TargetFreqency / PrivateConfig.AccTargetFreqency))
                {
                    _s_spiXfer(MPU6500_fd, Tmp_MPU6500_SPI_Buffer, Tmp_MPU6500_SPI_Bufferout, PrivateConfig.MPU6500_SPI_Freq, 8);
                    int Tmp_AX = (short)((int)Tmp_MPU6500_SPI_Bufferout[1] << 8 | (int)Tmp_MPU6500_SPI_Bufferout[2]);
                    int Tmp_AY = (short)((int)Tmp_MPU6500_SPI_Bufferout[3] << 8 | (int)Tmp_MPU6500_SPI_Bufferout[4]);
                    int Tmp_AZ = (short)((int)Tmp_MPU6500_SPI_Bufferout[5] << 8 | (int)Tmp_MPU6500_SPI_Bufferout[6]);

                    // Step 1: rotate Yaw
                    int Tmp_A2X = Tmp_AX * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_AY * sin(DEG2RAD((PrivateConfig.MPU_Flip___Yaw)));
                    int Tmp_A2Y = Tmp_AY * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_AX * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip___Yaw)));

                    // Step 2: rotate Pitch
                    int Tmp_A3X = Tmp_A2X * cos(DEG2RAD(PrivateConfig.MPU_Flip_Pitch)) + Tmp_AZ * sin(DEG2RAD((PrivateConfig.MPU_Flip_Pitch)));
                    int Tmp_A3Z = Tmp_AZ * cos(DEG2RAD((PrivateConfig.MPU_Flip_Pitch))) + Tmp_A2X * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip_Pitch)));
                    // Step 3: rotate Roll
                    PrivateData._uORB_MPU6500_A_Y = Tmp_A2Y * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_A3Z * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU6500_A_Z = Tmp_A3Z * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_A2Y * sin(DEG2RAD((PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU6500_A_X = Tmp_A3X;

                    //
                    PrivateData._uORB_MPU6500_AccelCountDown = 0;
                }
                PrivateData._uORB_MPU6500_AccelCountDown++;
                {
                    uint8_t Tmp_MPU6500_SPI_GBuffer[8] = {0};
                    uint8_t Tmp_MPU6500_SPI_GBufferout[8] = {0};
                    Tmp_MPU6500_SPI_GBuffer[0] = 0xC3;
                    _s_spiXfer(MPU6500_fd, Tmp_MPU6500_SPI_GBuffer, Tmp_MPU6500_SPI_GBufferout, PrivateConfig.MPU6500_SPI_Freq, 8);
                    int Tmp_GX = (short)((int)Tmp_MPU6500_SPI_GBufferout[1] << 8 | (int)Tmp_MPU6500_SPI_GBufferout[2]);
                    int Tmp_GY = (short)((int)Tmp_MPU6500_SPI_GBufferout[3] << 8 | (int)Tmp_MPU6500_SPI_GBufferout[4]);
                    int Tmp_GZ = (short)((int)Tmp_MPU6500_SPI_GBufferout[5] << 8 | (int)Tmp_MPU6500_SPI_GBufferout[6]);
                    // Step 1: rotate Yaw
                    int Tmp_G2X = Tmp_GX * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_GY * sin(DEG2RAD((PrivateConfig.MPU_Flip___Yaw)));
                    int Tmp_G2Y = Tmp_GY * cos(DEG2RAD((PrivateConfig.MPU_Flip___Yaw))) + Tmp_GX * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip___Yaw)));
                    // Step 2: rotate Pitch
                    int Tmp_G3X = Tmp_G2X * cos(DEG2RAD(PrivateConfig.MPU_Flip_Pitch)) + Tmp_GZ * sin(DEG2RAD((PrivateConfig.MPU_Flip_Pitch)));
                    int Tmp_G3Z = Tmp_GZ * cos(DEG2RAD((PrivateConfig.MPU_Flip_Pitch))) + Tmp_G2X * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip_Pitch)));
                    // Step 3: rotate Roll
                    PrivateData._uORB_MPU6500_G_Y = Tmp_G2Y * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_G3Z * sin(DEG2RAD((180 + PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU6500_G_Z = Tmp_G3Z * cos(DEG2RAD((PrivateConfig.MPU_Flip__Roll))) + Tmp_G2Y * sin(DEG2RAD((PrivateConfig.MPU_Flip__Roll)));
                    PrivateData._uORB_MPU6500_G_X = Tmp_G3X;

                    //
                }
            }
        }
    }

    inline int GetTimestamp()
    {
        struct timespec tv;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        return (((tv.tv_sec * (uint64_t)1000000 + (tv.tv_nsec / 1000))) - IMUstartuptime);
    }

    int MPU6500_fd;
    int IMUstartuptime = 0;
    int LastUpdate = 0;
    float MPU6500_Accel_LSB = MPU6500_ACCEL_LSB; //+-16g
    int _Tmp_AHRS_MAG_X = 0;
    int _Tmp_AHRS_MAG_Y = 0;
    int _Tmp_AHRS_MAG_Z = 0;
    bool AHRSEnable = false;

    MPUData PrivateData;
    MPUConfig PrivateConfig;

    pt1Filter_t VibeLPF[3];
    pt1Filter_t VibeFloorLPF[3];
    pt1Filter_t GryoFilterLPF[3];
    pt1Filter_t GryoFilterLPFST2[3];
    pt1Filter_t AccelFilterLPF[3];
    biquadFilter_t GryoFilterBLPFST2[3];
    biquadFilter_t GryoFilterBLPF[3];
    biquadFilter_t AccelFilterBLPF[3];
    biquadFilter_t GryoFilterDynamicNotch[3];
    biquadFilter_t GryoFilterDynamicFreq[3];
    biquadFilter_t GyroNotchFilter[3];
    biquadFilter_t AccelNotchFilter[3];
    std::unique_ptr<MadgwickAHRS> AHRSSys;
};