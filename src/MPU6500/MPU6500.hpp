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
float mpu_6500_GX;
float mpu_6500_GY;
float mpu_6500_GZ;
float mpu_6500_AX;
float mpu_6500_AY;
float mpu_6500_AZ;
float _mpu_6500_AX_;
float _mpu_6500_AY_;
float _mpu_6500_AZ_;

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
        usleep(3);
    }
    _mpu_6500_AX_ /= 2000;
    _mpu_6500_AY_ /= 2000;
    _mpu_6500_AZ_ /= 2000;
}

void SensorsParse()
{
    read_mpu_6500_data();
    mpu_6500_GX = (_mpu_6500_GX / 16384.0) * (9.8 * 2 / (22.5)) + 0.29;
    mpu_6500_GY = (_mpu_6500_GY / 16384.0) * (9.8 * 2 / (22.6)) - 0.26;
    mpu_6500_GZ = (_mpu_6500_GZ / 16384.0) * (9.8 * 2 / (22)) + 0.31;
    Angle_Pitch = asin((mpu_6500_GX) / 0.98) * 180 / 3.14;
    Angle_Roll = asin((mpu_6500_GY) / 0.98) * 180 / 3.14;

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
        
        float Angle_Roll_Out = 0.7 * Angle_Roll + 0.3 * Angle_Roll_last;
        float Angle_Pitch_Out = 0.7 * Angle_Pitch + 0.3 * Angle_Pitch_last;
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

