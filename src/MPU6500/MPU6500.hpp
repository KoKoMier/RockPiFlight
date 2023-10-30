#pragma once
#include <iostream>
#include "../SPI/LinuxSPI.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include <fstream>
#include "../SYS/sys.hpp"

float pi = 3.14159265358979323;
float MPU_6500_LSB = 65.5 / 4;
//
int _mpu_6500_GX;
int _mpu_6500_GY;
int _mpu_6500_GZ;
int _mpu_6500_AX;
int _mpu_6500_AY;
int _mpu_6500_AZ;

float Angle_Roll_Gyro_ = 0;
float Angle_Pitch_Gyro_ = 0;

float _mpu_6500_AZ_err_up = 0;
float _mpu_6500_AZ_err_down = 0;
float _mpu_6500_AX_err_up = 0;
float _mpu_6500_AX_err_down = 0;
float _mpu_6500_AY_err_up = 0;
float _mpu_6500_AY_err_down = 0;

float Angle_Roll_Out;
float Angle_Pitch_Out;

float mpu_6500_GX_Now = 0.0;
float mpu_6500_GY_Now = 0.0;
float mpu_6500_GZ_Now = 0.0;
float mpu_6500_GX_Last = 0.0;
float mpu_6500_GY_Last = 0.0;
float mpu_6500_GZ_Last = 0.0;

float mpu_6500_GX; // x轴角速度
float mpu_6500_GY; // y轴角速度
float mpu_6500_GZ; // z轴角速度
float mpu_6500_AX; // x轴加速度
float mpu_6500_AY; // y轴加速度
float mpu_6500_AZ; // z轴加速度

float _mpu_6500_AX_Cali; // x轴加速度校准值
float _mpu_6500_AY_Cali; // y轴加速度校准值
float _mpu_6500_AZ_Cali; // z轴加速度校准值

float _mpu_6500_GX_Cali = 0.0; // x轴角速度校准值
float _mpu_6500_GY_Cali = 0.0; // y轴角速度校准值
float _mpu_6500_GZ_Cali = 0.0; // z轴角速度校准值

float Angle_Pitch_Acc_Cali = 0.0;  // 校准值
float Angle_Roll_Acc_Cali = 0.0;   // 校准值
float Acc_Total_Vector_Cali = 0.0; // 校准值
//
float Angle_Pitch = 0.0;
float Angle_Roll = 0.0;
float Angle_Yaw = 0.0;

float Angle_Pitch_Gyro = 0.0;
float Angle_Roll_Gyro = 0.0;
float Angle_Yaw_Gyro = 0.0;

float Angle_Pitch_Acc = 0.0;
float Angle_Roll_Acc = 0.0;
float Angle_Yaw_Acc = 0.0;
float Acc_Total_Vector = 0.0;

//
bool set_groy_angles = false;
float Angle_Roll_last = 0;
float Angle_Pitch_last = 0;

float MPU6500_SPI_Freq = 400000;
//
float Angle_Pitch_Gyro_last = 0;
float Angle_Roll_Gyro_last = 0;
float Angle_Yaw_Gyro_last = 0;

int fd = _s_spiOpen("/dev/spidev0.0", MPU6500_SPI_Freq, 0);
int count = 0;

void read_mpu_6500_data()
{
    uint8_t Tmp_MPU6500_SPI_BufferX[8] = {0};

    //
    Tmp_MPU6500_SPI_BufferX[0] = 0xBA;
    _s_spiXfer(fd, Tmp_MPU6500_SPI_BufferX, Tmp_MPU6500_SPI_BufferX, MPU6500_SPI_Freq, 8);
    int Tmp_AX = (short)((int)Tmp_MPU6500_SPI_BufferX[1 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[2 + 1]);
    int Tmp_AY = (short)((int)Tmp_MPU6500_SPI_BufferX[3 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[4 + 1]);
    int Tmp_AZ = (short)((int)Tmp_MPU6500_SPI_BufferX[5 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[6 + 1]);
    _mpu_6500_AX = Tmp_AX;
    _mpu_6500_AY = Tmp_AY;
    _mpu_6500_AZ = Tmp_AZ;
    // std::cout << "_mpu_6500_AX:" << _mpu_6500_AX << "\r\n";

    Tmp_MPU6500_SPI_BufferX[0] = 0xC3;
    _s_spiXfer(fd, Tmp_MPU6500_SPI_BufferX, Tmp_MPU6500_SPI_BufferX, MPU6500_SPI_Freq, 8);
    int Tmp_GX = (short)((int)Tmp_MPU6500_SPI_BufferX[1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[2]);
    int Tmp_GY = (short)((int)Tmp_MPU6500_SPI_BufferX[3] << 8 | (int)Tmp_MPU6500_SPI_BufferX[4]);
    int Tmp_GZ = (short)((int)Tmp_MPU6500_SPI_BufferX[5] << 8 | (int)Tmp_MPU6500_SPI_BufferX[6]);
    _mpu_6500_GX = Tmp_GX;
    _mpu_6500_GY = Tmp_GY;
    _mpu_6500_GZ = Tmp_GZ;
}

void SensorsAcorrect()
{
    uint8_t MPU6500_SPI_Config_RESET[2] = {0x6b, 0x80};
    _s_spiWrite(fd, MPU6500_SPI_Config_RESET, MPU6500_SPI_Freq, 2); // reset
    usleep(500);
    uint8_t MPU6500_SPI_Config_RESET2[2] = {0x68, 0x07};
    _s_spiWrite(fd, MPU6500_SPI_Config_RESET2, MPU6500_SPI_Freq, 2); //
    usleep(500);
    uint8_t MPU6500_SPI_Config_RESET3[2] = {0x6b, 0x00};
    _s_spiWrite(fd, MPU6500_SPI_Config_RESET3, MPU6500_SPI_Freq, 2); //
    usleep(500);
    uint8_t MPU6500_SPI_Config_RESET4[2] = {0x6b, 0x01};
    _s_spiWrite(fd, MPU6500_SPI_Config_RESET4, MPU6500_SPI_Freq, 2); //
    usleep(1000);

    uint8_t MPU6500_SPI_Config_ALPF[2] = {0x1d, 0x00};
    _s_spiWrite(fd, MPU6500_SPI_Config_ALPF, MPU6500_SPI_Freq, 2); //
    usleep(15);
    uint8_t MPU6500_SPI_Config_Acce[2] = {0x1c, 0x18};
    _s_spiWrite(fd, MPU6500_SPI_Config_Acce, MPU6500_SPI_Freq, 2); //
    usleep(15);
    uint8_t MPU6500_SPI_Config_Gyro[2] = {0x1b, 0x18};
    _s_spiWrite(fd, MPU6500_SPI_Config_Gyro, MPU6500_SPI_Freq, 2); //
    usleep(15);
    uint8_t MPU6500_SPI_Config_GLPF[2] = {0x1a, 0x00};
    _s_spiWrite(fd, MPU6500_SPI_Config_GLPF, MPU6500_SPI_Freq, 2); //
    usleep(15);

    uint8_t MPU6500_SPI_Config_INTC[2] = {0x37, 0x22};
    _s_spiWrite(fd, MPU6500_SPI_Config_INTC, MPU6500_SPI_Freq, 2);
    usleep(500);
    uint8_t MPU6500_SPI_Config_INTE[2] = {0x38, 0x01};
    _s_spiWrite(fd, MPU6500_SPI_Config_INTE, MPU6500_SPI_Freq, 2);
    usleep(500);

    std::ifstream file("./data/example.txt");
    if (!file)
    {
        std::cout << "无法打开文件\n";
        return;
    }

    float error[6];
    std::string line;
    for (int i = 0; i < 6; i++)
    {
        if (std::getline(file, line))
        {
            std::size_t pos = line.find(":");
            if (pos != std::string::npos)
            {
                std::string value_str = line.substr(pos + 1);
                double value = std::stod(value_str);
                error[i] = value;
            }
        }
    }

    file.close();
    _mpu_6500_AZ_err_up = error[0];
    _mpu_6500_AZ_err_down = error[1];
    _mpu_6500_AX_err_up = error[2];
    _mpu_6500_AX_err_down = error[3];
    _mpu_6500_AY_err_up = error[4];
    _mpu_6500_AY_err_down = error[5];
    // std::cout << "error: " << _mpu_6500_AZ_err_up << "\n";
    // std::cout << "error: " << _mpu_6500_AZ_err_down << "\n";
    // std::cout << "error: " << _mpu_6500_AX_err_up << "\n";
    // std::cout << "error: " << _mpu_6500_AX_err_down << "\n";
    // std::cout << "error: " << _mpu_6500_AY_err_up << "\n";
    // std::cout << "error: " << _mpu_6500_AY_err_down << "\n";

    for (int i = 0; i < 2000; i++)
    {
        read_mpu_6500_data();

        _mpu_6500_GX_Cali += _mpu_6500_GX;
        _mpu_6500_GY_Cali += _mpu_6500_GY;
        _mpu_6500_GZ_Cali += _mpu_6500_GZ;

        _mpu_6500_AX_Cali += _mpu_6500_AX;
        _mpu_6500_AY_Cali += _mpu_6500_AY;
        _mpu_6500_AZ_Cali += _mpu_6500_AZ;

        usleep(300);
    }
    // std::cout << "mpu_6500_GX:" << _mpu_6500_AY__ << "\r\n";
    _mpu_6500_GX_Cali = _mpu_6500_GX_Cali / 2000;
    _mpu_6500_GY_Cali = _mpu_6500_GY_Cali / 2000;
    _mpu_6500_GZ_Cali = _mpu_6500_GZ_Cali / 2000;

    _mpu_6500_AX_Cali = _mpu_6500_AX_Cali / 2000;
    _mpu_6500_AY_Cali = _mpu_6500_AY_Cali / 2000;
    _mpu_6500_AZ_Cali = _mpu_6500_AZ_Cali / 2000;

    // std::cout << "mpu_6500_AX:" << _mpu_6500_AX_Cali << "\r\n";
    // std::cout << "mpu_6500_AY:" << _mpu_6500_AY_Cali << "\r\n";
    // std::cout << "mpu_6500_AZ:" << _mpu_6500_AZ_Cali << "\r\n";
}

void SensorsParse()
{
    read_mpu_6500_data();

    mpu_6500_GX_Now = (_mpu_6500_GX - _mpu_6500_GX_Cali) / MPU_6500_LSB;
    mpu_6500_GY_Now = (_mpu_6500_GY - _mpu_6500_GY_Cali) / MPU_6500_LSB;
    mpu_6500_GZ_Now = (_mpu_6500_GZ - _mpu_6500_GZ_Cali) / MPU_6500_LSB;

    // 滤波得到角速度
    mpu_6500_GX = (0.2 * mpu_6500_GX_Now + 0.8 * mpu_6500_GX_Last);
    mpu_6500_GY = (0.2 * mpu_6500_GY_Now + 0.8 * mpu_6500_GY_Last);
    mpu_6500_GZ = (0.2 * mpu_6500_GZ_Now + 0.8 * mpu_6500_GZ_Last);

    mpu_6500_GX_Last = mpu_6500_GX;
    mpu_6500_GY_Last = mpu_6500_GY;
    mpu_6500_GZ_Last = mpu_6500_GZ;
    // std::cout << "GZ" << mpu_6500_GX << "\r\n";

    Angle_Pitch_Gyro -= mpu_6500_GX / TF.UpdateFreq;
    Angle_Roll_Gyro -= mpu_6500_GY / TF.UpdateFreq;
    Angle_Yaw_Gyro -= mpu_6500_GZ / TF.UpdateFreq;

    // std::cout << "Angle_Pitch_Gyro:" << Angle_Pitch_Gyro << "\r\n";

    if (std::isnan(Angle_Pitch_Gyro))
    {
        Angle_Pitch_Gyro = Angle_Pitch_Gyro_last;
    }
    if (std::isnan(Angle_Roll_Gyro))
    {
        Angle_Roll_Gyro = Angle_Roll_Gyro_last;
    }

    Angle_Pitch_Gyro_last = Angle_Pitch_Gyro;
    Angle_Roll_Gyro_last = Angle_Roll_Gyro;
    Angle_Yaw_Gyro_last = Angle_Yaw_Gyro;
    //  std::cout << "Angle_Yaw_Gyro:" << Angle_Yaw_Gyro << "\r\n";
    // std::cout << "Angle_Pitch:" << Angle_Pitch_Gyro << "\r\n";
    // std::cout << "Angle_Roll:" << Angle_Roll_Gyro << "\r\n";

    Angle_Pitch_Gyro += Angle_Roll_Gyro * sin((mpu_6500_GZ / TF.UpdateFreq) * pi / 180.0);
    Angle_Roll_Gyro -= Angle_Pitch_Gyro * sin((mpu_6500_GZ / TF.UpdateFreq) * pi / 180.0);

    // std::cout << "mpu_6500_GX_Now:" << mpu_6500_GX << "\r\n";
    //  std::cout << "Angle_Yaw_Gyro:" << Angle_Yaw_Gyro << "\r\n";
    // std::cout << "Angle_Pitch:" << Angle_Pitch_Gyro << "\r\n";
    // std::cout << "Angle_Roll:" << Angle_Roll_Gyro << "\r\n";

    // std::cout << "_mpu_6500_AX:" << mpu_6500_AX << "\r\n";
    // std::cout << "_mpu_6500_AY:" << mpu_6500_AY << "\r\n";
    // std::cout << "_mpu_6500_AZ:" << mpu_6500_AZ << "\r\n";

    mpu_6500_AX = (_mpu_6500_AX - _mpu_6500_AX_err_up) - (_mpu_6500_AX_err_down - _mpu_6500_AX_err_up) / 2;
    mpu_6500_AY = (_mpu_6500_AY - _mpu_6500_AY_err_up) - (_mpu_6500_AY_err_down - _mpu_6500_AY_err_up) / 2;
    mpu_6500_AZ = (_mpu_6500_AZ - _mpu_6500_AZ_err_up) - (_mpu_6500_AZ_err_down - _mpu_6500_AZ_err_up) / 2;

    // std::cout << "_mpu_6500_AX:" << (_mpu_6500_AX + 7030) / 2000.0 << "\r\n";
    // std::cout << "_mpu_6500_AY:" << _mpu_6500_AY << "\r\n";
    //  std::cout << "_mpu_6500_AZ:" << _mpu_6500_AZ + 32768 << "\r\n";
    // _mpu_6500_AX = _mpu_6500_AX > -7000.0 ? -7000.0 : (_mpu_6500_AX < -9000.0 ? -9000.0 : _mpu_6500_AX);
    // _mpu_6500_AY = _mpu_6500_AY > -6000.0 ? -6000.0 : (_mpu_6500_AY < -10000.0 ? -10000.0 : _mpu_6500_AY);
    // Angle_Roll_Acc = -asin((_mpu_6500_AX + 7030) / 2000.0) * 180.0 / pi;
    // Angle_Pitch_Acc = asin((_mpu_6500_AY + 8038) / 2000.0) * 180.0 / pi;
    Acc_Total_Vector = sqrt((mpu_6500_AX * mpu_6500_AX) + (mpu_6500_AY * mpu_6500_AY) + (mpu_6500_AZ * mpu_6500_AZ));
    Angle_Pitch_Acc = asin((float)mpu_6500_AY / Acc_Total_Vector) * 180 / pi;
    Angle_Roll_Acc = asin((float)mpu_6500_AX / Acc_Total_Vector) * 180 / pi * (-1);

    // Angle_Pitch_Acc -= Angle_Pitch_Acc_Cali;
    // Angle_Roll_Acc -= Angle_Roll_Acc_Cali;

    // std::cout << "Acc_Total_Vector:" << Acc_Total_Vector << "\r\n";
    // std::cout << "Angle_Pitch_Acc:" << Angle_Roll_Acc << "\r\n";
    // std::cout << "Angle_Roll_Acc:" << Angle_Pitch_Acc << "\r\n";
    //   std::cout << std::endl;

    if (set_groy_angles)
    {

        // std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(5) << Angle_Pitch_Gyro << "\r\n";
        // std::cout << "Angle_Roll:" << std::fixed << std::setprecision(5) << Angle_Roll_Gyro << "\r\n";

        Angle_Roll_Gyro = Angle_Roll_Gyro * 0.999 + Angle_Roll_Acc * 0.001;
        Angle_Pitch_Gyro = Angle_Pitch_Gyro * 0.999 + Angle_Pitch_Acc * 0.001;

        Angle_Roll_Out = 0.85 * Angle_Roll_Gyro + 0.15 * Angle_Roll_last;
        Angle_Pitch_Out = 0.85 * Angle_Pitch_Gyro + 0.15 * Angle_Pitch_last;

        if (std::isnan(Angle_Pitch_Out))
        {
            Angle_Pitch_Out = Angle_Pitch_last;
        }
        if (std::isnan(Angle_Roll_Out))
        {
            Angle_Roll_Out = Angle_Roll_last;
        }

        // std::cout << "Angle_Roll_Out:" << std::fixed << std::setprecision(1) << Angle_Roll_Out << "\r\n";
        // std::cout << "Angle_Pitch_Out:" << std::fixed << std::setprecision(1) << Angle_Pitch_Out << "\r\n";

        // std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(1) << Angle_Pitch_Gyro << "\r\n";
        // std::cout << "Angle_Roll:" << std::fixed << std::setprecision(1) << Angle_Roll_Gyro << "\r\n";

        Angle_Roll_last = Angle_Roll_Out;
        Angle_Pitch_last = Angle_Pitch_Out;
    }
    else
    {
        Angle_Roll_Gyro = Angle_Roll_Acc;
        Angle_Pitch_Gyro = Angle_Pitch_Acc;
        set_groy_angles = true;
    }

    // std::cout << "count" << count << "\r\n";
    //  Angle_Pitch = Angle_Pitch_Gyro * 0.996 + Angle_Pitch_Acc * 0.004;
    //  Angle_Roll = Angle_Roll_Gyro * 0.996 + Angle_Roll_Acc * 0.004;

    // std::cout << "Angle_Pitch:" << Angle_Pitch << "\r\n";
    // std::cout << "Angle_Roll:" << Angle_Roll << "\r\n";

    // std::cout << "mpu_6500_GX:" << mpu_6500_GX << "\r\n";
    // std::cout << "mpu_6500_GY:" << mpu_6500_GY << "\r\n";
    // std::cout << "mpu_6500_GZ:" << _mpu_6500_GY_ << "\r\n";

    // float gx = mpu_6500_GX / 9.8;
    // gx = gx > 1.0 ? 1.0 : (gx < -1.0 ? -1.0 : gx);
    // Angle_Roll = asin(gx) * 180 / 3.14;

    // float gy = mpu_6500_GY / 9.8;
    // gy = gy > 1.0 ? 1.0 : (gy < -1.0 ? -1.0 : gy);
    // Angle_Pitch = asin(gy) * 180 / 3.14;

    // std::cout << "Angle_Pitch:" << Angle_Pitch << "\r\n";
    // std::cout << "Angle_Roll:" << Angle_Roll << "\r\n";

    // mpu_6500_AX = _mpu_6500_AX - _mpu_6500_AX_;
    // mpu_6500_AY = _mpu_6500_AY - _mpu_6500_AY_;
    // mpu_6500_AZ = _mpu_6500_AZ - _mpu_6500_AZ_;

    // float Acc_Total_Vector = sqrt((mpu_6500_AX * mpu_6500_AX) + (mpu_6500_AY * mpu_6500_AY) + (mpu_6500_AZ * mpu_6500_AZ));
    // float Angle_pitch_Acc = asin((float)mpu_6500_AY / Acc_Total_Vector) * 180 / 3.14;
    // float Angle_roll_Acc = asin((float)mpu_6500_AX / Acc_Total_Vector) * 180 / 3.14 * (-1);

    // std::cout << "mpu_6500_AX:" << mpu_6500_AX << "\r\n";
    // std::cout << "mpu_6500_AY:" << mpu_6500_AY << "\r\n";
    // std::cout << "mpu_6500_AZ:" << mpu_6500_AZ << "\r\n";
    // std::cout << "_mpu_6500_AX_:" << _mpu_6500_AX_ << "\r\n";
    // std::cout << "_mpu_6500_AY_:" << _mpu_6500_AY_ << "\r\n";
    // std::cout << "_mpu_6500_AZ_:" << _mpu_6500_AZ_ << "\r\n";
    // std::cout << "Angle_Pitch:" << Angle_pitch_Acc << "\r\n";
    // std::cout << "Angle_Roll:" << Angle_roll_Acc << "\r\n";
    // std::cout << "Acc_Total_Vector:" << Acc_Total_Vector << "\r\n";

    // if (set_groy_angles)
    // {
    //     Angle_Roll = Angle_Roll * 0.9996 + Angle_roll_Acc * 0.0004;
    //     Angle_Pitch = Angle_Pitch * 0.9996 + Angle_pitch_Acc * 0.0004;
    //     Angle_Yaw += mpu_6500_AY;

    //     Angle_Roll_Out = 0.7 * Angle_Roll + 0.3 * Angle_Roll_last;
    //     Angle_Pitch_Out = 0.7 * Angle_Pitch + 0.3 * Angle_Pitch_last;
    //     Angle_Roll_last = Angle_Roll_Out;
    //     Angle_Pitch_last = Angle_Pitch_Out;
    // }
    // else
    // {
    //     Angle_Roll = Angle_roll_Acc;
    //     Angle_Pitch = Angle_pitch_Acc;
    //     set_groy_angles = true;
    // }
}

