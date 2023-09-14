#include <iostream>
#include "../spi/LinuxSPI.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>

float MPU_6500_LSB = 65.5 * 2.0;
float UpdateFreq = 1000000.0 / 8000.0;//运行频率
float x_gain = 1.0 / 104.0 * 90.0;
//
int _mpu_6500_GX;
int _mpu_6500_GY;
int _mpu_6500_GZ;
int _mpu_6500_AX;
int _mpu_6500_AY;
int _mpu_6500_AZ;

float Angle_Roll_Out;
float Angle_Pitch_Out;
float mpu_6500_GX_Now = 0.0;
float mpu_6500_GY_Now = 0.0;
float mpu_6500_GZ_Now = 0.0;
float mpu_6500_GX_Last = 0.0;
float mpu_6500_GY_Last = 0.0;
float mpu_6500_GZ_Last = 0.0;
float mpu_6500_GX;//x轴角速度
float mpu_6500_GY;//y轴角速度
float mpu_6500_GZ;//z轴角速度
float mpu_6500_AX;//x轴加速度
float mpu_6500_AY;//y轴加速度
float mpu_6500_AZ;//z轴加速度
float _mpu_6500_AX_;//加速度初始值
float _mpu_6500_AY_;//加速度初始值
float _mpu_6500_AZ_;//加速度初始值
float _mpu_6500_GX_;//x轴角速度校准值
float _mpu_6500_GY_;//y轴角速度校准值
float _mpu_6500_GZ_;//z轴角速度校准值
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

//
int fd = _s_spiOpen("/dev/spidev1.0", 400000, 0);

void read_mpu_6500_data()
{
    uint8_t Tmp_MPU6500_SPI_BufferX[8] = {0};
    Tmp_MPU6500_SPI_BufferX[0] = 0x1C;
    Tmp_MPU6500_SPI_BufferX[1] = 0x00;
    _s_spiWrite(fd, Tmp_MPU6500_SPI_BufferX, 400000, 3);
    //
    Tmp_MPU6500_SPI_BufferX[0] = 0xBA;
    _s_spiXfer(fd, Tmp_MPU6500_SPI_BufferX, Tmp_MPU6500_SPI_BufferX, 400000, 8);
    int Tmp_AX = (short)((int)Tmp_MPU6500_SPI_BufferX[1 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[2 + 1]);
    int Tmp_AY = (short)((int)Tmp_MPU6500_SPI_BufferX[3 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[4 + 1]);
    int Tmp_AZ = (short)((int)Tmp_MPU6500_SPI_BufferX[5 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[6 + 1]);
    _mpu_6500_AX = Tmp_AX;
    _mpu_6500_AY = Tmp_AY;
    _mpu_6500_AZ = Tmp_AZ;

    Tmp_MPU6500_SPI_BufferX[0] = 0xC3;
    _s_spiXfer(fd, Tmp_MPU6500_SPI_BufferX, Tmp_MPU6500_SPI_BufferX, 400000, 8);
    int Tmp_GX = (short)((int)Tmp_MPU6500_SPI_BufferX[1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[2]);
    int Tmp_GY = (short)((int)Tmp_MPU6500_SPI_BufferX[3] << 8 | (int)Tmp_MPU6500_SPI_BufferX[4]);
    int Tmp_GZ = (short)((int)Tmp_MPU6500_SPI_BufferX[5] << 8 | (int)Tmp_MPU6500_SPI_BufferX[6]);
    _mpu_6500_GX = Tmp_GX;
    _mpu_6500_GY = Tmp_GY;
    _mpu_6500_GZ = Tmp_GZ;
}

void SensorsAcorrect()
{
    for (int i = 0; i < 2000; i++)
    {
        read_mpu_6500_data();

        _mpu_6500_GX_ += _mpu_6500_GX;
        _mpu_6500_GY_ += _mpu_6500_GY;
        _mpu_6500_GZ_ += _mpu_6500_GZ;

        usleep(300);
    }
    // std::cout << "mpu_6500_GX:" << _mpu_6500_AY__ << "\r\n";
    _mpu_6500_GX_ /= 2000;
    _mpu_6500_GY_ /= 2000;
    _mpu_6500_GZ_ /= 2000;
}

void SensorsParse()
{
    read_mpu_6500_data();

    mpu_6500_GX_Now = _mpu_6500_GX - _mpu_6500_GX_;
    mpu_6500_GY_Now = _mpu_6500_GY - _mpu_6500_GY_;
    mpu_6500_GZ_Now = _mpu_6500_GZ - _mpu_6500_GZ_;

    //滤波得到角速度
    mpu_6500_GX = (0.3 * mpu_6500_GX_Now / MPU_6500_LSB + 0.7 * mpu_6500_GX_Last);
    mpu_6500_GY = (0.3 * mpu_6500_GY_Now / MPU_6500_LSB + 0.7 * mpu_6500_GY_Last);
    mpu_6500_GZ = (0.3 * mpu_6500_GZ_Now / MPU_6500_LSB + 0.7 * mpu_6500_GZ_Last);

    mpu_6500_GX_Last = mpu_6500_GX;
    mpu_6500_GY_Last = mpu_6500_GY;
    mpu_6500_GZ_Last = mpu_6500_GZ;

    Angle_Pitch_Gyro += mpu_6500_GX_Now / MPU_6500_LSB / UpdateFreq;
    Angle_Roll_Gyro += mpu_6500_GY_Now / MPU_6500_LSB / UpdateFreq;
    Angle_Pitch_Gyro += Angle_Roll_Gyro * sin((mpu_6500_GZ / UpdateFreq) * (3.14 / 180.0));
    Angle_Roll_Gyro -= Angle_Pitch_Gyro * sin((mpu_6500_GZ / UpdateFreq) * (3.14 / 180.0));

    // std::cout << "Angle_Pitch:" << Angle_Pitch_Gyro << "\r\n";
    // std::cout << "Angle_Roll:" << Angle_Roll_Gyro << "\r\n";
    // std::cout << "GZ" << mpu_6500_GZ / MPU_6500_LSB / UpdateFreq << "\r\n";
    

    
    mpu_6500_AX = _mpu_6500_AX;
    mpu_6500_AY = _mpu_6500_AY;
    mpu_6500_AZ = _mpu_6500_AZ;

    Acc_Total_Vector = sqrt((mpu_6500_AX * mpu_6500_AX) + (mpu_6500_AY * mpu_6500_AY) + (mpu_6500_AZ * mpu_6500_AZ));
    Angle_Pitch_Acc = asin((float)mpu_6500_AY / Acc_Total_Vector) * 180 / 3.14;
    Angle_Roll_Acc = asin((float)mpu_6500_AX / Acc_Total_Vector) * 180 / 3.14 * (-1);

    // std::cout << "Angle_Pitch_Acc:" << Angle_Pitch_Acc << "\r\n";
    // std::cout << "Angle_Roll_Acc:" << Angle_Roll_Acc << "\r\n";

    Angle_Pitch = Angle_Pitch_Gyro * 0.996 + Angle_Pitch_Acc * 0.004;
    Angle_Roll = Angle_Roll_Gyro * 0.996 + Angle_Roll_Acc * 0.004;

    std::cout << "Angle_Pitch:" << Angle_Pitch << "\r\n";
    std::cout << "Angle_Roll:" << Angle_Roll << "\r\n";

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
