#include <iostream>
#include "../spi/LinuxSPI.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>

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
float mpu_6500_GX;
float mpu_6500_GY;
float mpu_6500_GZ;
float mpu_6500_AX;
float mpu_6500_AY;
float mpu_6500_AZ;
float _mpu_6500_AX_;
float _mpu_6500_AY_;
float _mpu_6500_AZ_;
float _mpu_6500_AX__;
float _mpu_6500_AY__;
float _mpu_6500_AZ__;
//
float Angle_Pitch;
float Angle_Roll;
float Angle_Yaw;

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
    int Tmp_GX = (short)((int)Tmp_MPU6500_SPI_BufferX[1 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[2 + 1]);
    int Tmp_GY = (short)((int)Tmp_MPU6500_SPI_BufferX[3 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[4 + 1]);
    int Tmp_GZ = (short)((int)Tmp_MPU6500_SPI_BufferX[5 + 1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[6 + 1]);
    _mpu_6500_GX = Tmp_GX;
    _mpu_6500_GY = Tmp_GY;
    _mpu_6500_GZ = Tmp_GZ;

    Tmp_MPU6500_SPI_BufferX[0] = 0xC3;
    _s_spiXfer(fd, Tmp_MPU6500_SPI_BufferX, Tmp_MPU6500_SPI_BufferX, 400000, 8);
    int Tmp_AX = (short)((int)Tmp_MPU6500_SPI_BufferX[1] << 8 | (int)Tmp_MPU6500_SPI_BufferX[2]);
    int Tmp_AY = (short)((int)Tmp_MPU6500_SPI_BufferX[3] << 8 | (int)Tmp_MPU6500_SPI_BufferX[4]);
    int Tmp_AZ = (short)((int)Tmp_MPU6500_SPI_BufferX[5] << 8 | (int)Tmp_MPU6500_SPI_BufferX[6]);
    _mpu_6500_AX = Tmp_AX;
    _mpu_6500_AY = Tmp_AY;
    _mpu_6500_AZ = Tmp_AZ;
}

void SensorsAcorrect()
{
    for (int i = 0; i < 2000; i++)
    {
        read_mpu_6500_data();
        _mpu_6500_AX_ += _mpu_6500_AX;
        _mpu_6500_AY_ += _mpu_6500_AY;
        _mpu_6500_AZ_ += _mpu_6500_AZ;

        mpu_6500_GX_Now = ((_mpu_6500_GX / 16384.0) + 1.5) * 19.6 / 2.24 - 9.605;
        mpu_6500_GY_Now = ((_mpu_6500_GY / 16384.0) + 0.84) * 19.6 / 2.24 - 10.07;
        mpu_6500_GZ_Now = ((_mpu_6500_GZ / 16384.0) + 1.5) * 19.6 / 2.24 - 9.8;

        _mpu_6500_AX__ += mpu_6500_GX_Now;
        _mpu_6500_AY__ += mpu_6500_GY_Now;
        _mpu_6500_AZ__ += mpu_6500_GZ_Now;

        usleep(3);
    }
    // std::cout << "mpu_6500_GX:" << _mpu_6500_AY__ << "\r\n";
    _mpu_6500_AX_ /= 2000;
    _mpu_6500_AY_ /= 2000;
    _mpu_6500_AZ_ /= 2000;
    _mpu_6500_AX__ /= 2000;
    _mpu_6500_AY__ /= 2000;
    _mpu_6500_AZ__ /= 2000;
}

void SensorsParse()
{
    read_mpu_6500_data();

    mpu_6500_GX_Now = ((_mpu_6500_GX / 16384.0) + 1.5) * 19.6 / 2.24 - 9.605;
    mpu_6500_GY_Now = ((_mpu_6500_GY / 16384.0) + 0.84) * 19.6 / 2.24 - 10.07;
    mpu_6500_GZ_Now = ((_mpu_6500_GZ / 16384.0) + 1.5) * 19.6 / 2.24 - 9.8;

    mpu_6500_GX = (0.3 * mpu_6500_GX_Now + 0.7 * mpu_6500_GX_Last);
    mpu_6500_GY = (0.3 * mpu_6500_GY_Now + 0.7 * mpu_6500_GY_Last);
    mpu_6500_GZ = (0.3 * mpu_6500_GZ_Now + 0.7 * mpu_6500_GZ_Last);

    mpu_6500_GX_Last = mpu_6500_GX;
    mpu_6500_GY_Last = mpu_6500_GY;
    mpu_6500_GZ_Last = mpu_6500_GZ;

    mpu_6500_GX -= _mpu_6500_AX__;
    mpu_6500_GY -= _mpu_6500_AY__;
    mpu_6500_GZ -= _mpu_6500_AZ__;
    //std::cout << "mpu_6500_GX:" << mpu_6500_GX << "\r\n";
    //std::cout << "mpu_6500_GY:" << mpu_6500_GY << "\r\n";
    //std::cout << "mpu_6500_GZ:" << _mpu_6500_AY__ << "\r\n";

    float gx = mpu_6500_GX / 9.8;
    gx = gx > 1.0 ? 1.0 : (gx < -1.0 ? -1.0 : gx);
    Angle_Roll = asin(gx) * 180 / 3.14;

    float gy = mpu_6500_GY / 9.8;
    gy = gy > 1.0 ? 1.0 : (gy < -1.0 ? -1.0 : gy);
    Angle_Pitch = asin(gy) * 180 / 3.14;

    //std::cout << "Angle_Pitch:" << Angle_Pitch << "\r\n";
    //std::cout << "Angle_Roll:" << Angle_Roll << "\r\n";

    mpu_6500_AX = _mpu_6500_AX - _mpu_6500_AX_;
    mpu_6500_AY = _mpu_6500_AY - _mpu_6500_AY_;
    mpu_6500_AZ = _mpu_6500_AZ - _mpu_6500_AZ_;

    float Acc_Total_Vector = sqrt((mpu_6500_AX * mpu_6500_AX) + (mpu_6500_AY * mpu_6500_AY) + (mpu_6500_AZ * mpu_6500_AZ));
    float Angle_pitch_Acc = asin((float)mpu_6500_AY / Acc_Total_Vector) * 180 / 3.14;
    float Angle_roll_Acc = asin((float)mpu_6500_AX / Acc_Total_Vector) * 180 / 3.14 * (-1);

    if (set_groy_angles)
    {
        Angle_Roll = Angle_Roll * 0.9996 + Angle_roll_Acc * 0.0004;
        Angle_Pitch = Angle_Pitch * 0.9996 + Angle_pitch_Acc * 0.0004;
        Angle_Yaw += mpu_6500_AY;

        Angle_Roll_Out = 0.7 * Angle_Roll + 0.3 * Angle_Roll_last;
        Angle_Pitch_Out = 0.7 * Angle_Pitch + 0.3 * Angle_Pitch_last;
        Angle_Roll_last = Angle_Roll_Out;
        Angle_Pitch_last = Angle_Pitch_Out;
    }
    else
    {
        Angle_Roll = Angle_roll_Acc;
        Angle_Pitch = Angle_pitch_Acc;
        set_groy_angles = true;
    }
}
