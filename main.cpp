#include <iostream>
#include "src/MPU6500/MPU6500.hpp"
#include "src/spi/LinuxSPI.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include "src/PAC9685/pac.hpp"
#include "src/UART/CRSFProtocol.hpp"
#include "src/UART/uart.hpp"
#include <sys/time.h>
#include <thread>


float pid_p_gain_roll = 0;
float pid_i_gain_roll = 0;
float pid_d_gain_roll = 0;

float pid_p_gain_pitch = 0;
float pid_i_gain_pitch = 0;
float pid_d_gain_pitch = 0;


int signalIn = 0;
int TimestartUpLoad = 0;

float pid_roll_error = 0;
float gyro_roll_input = 0;
float pid_roll_setpoint = 0;
float pid_i_sum_roll = 0;
float pid_output_roll = 0;
float pid_last_d_err = 0;

float pid_pitch_error = 0;
float gyro_pitch_input = 0;
float pid_pitch_setpoint = 0;
float pid_i_sum_pitch = 0;
float pid_output_pitch = 0;
float pid_last_d_err_pitch = 0;

int pin1 = 11;
int pin2 = 15;
int pin3 = 14;
int pin4 = 12;
void dataParese(std::string data, std::string *databuff, const char splti, int MaxSize)
{
    std::istringstream f(data);
    std::string s;
    int count = 0;
    while (getline(f, s, splti))
    {
        if (count > MaxSize)
            break;
        databuff[count] = s;
        count++;
    }
}

float limitValue(float value) {
    if (value > 3200) {
        return 3200;
    } else {
        return value;
    }
}

int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return ((tv.tv_sec * (uint64_t)1000000 + tv.tv_usec));
}

int main(int argc, char *argv[])
{
    int argvs;
    TimestartUpLoad = GetTimestamp();

    while ((argvs = getopt(argc, argv, "s::g::r::c::f::R")) != -1)
    {
        switch (argvs)
        {

        case 's':
        {
            SensorsAcorrect();
            while (true)
            {
                SensorsParse();
                //
                std::cout << "---------------------------"
                        << "\r\n";
                std::cout << "mpu_6500_GX:" << std::fixed << std::setprecision(5) << mpu_6500_GX << "\r\n";
                std::cout << "mpu_6500_GY:" << std::fixed << std::setprecision(5) << mpu_6500_GY << "\r\n";
                std::cout << "mpu_6500_GZ:" << std::fixed << std::setprecision(5) << mpu_6500_GZ << "\r\n";
                std::cout << "---------------------------"
                        << "\r\n";
                std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(1) << Angle_Pitch << "\r\n";
                std::cout << "Angle_Roll:" << std::fixed << std::setprecision(1) << Angle_Roll << "\r\n";
                std::cout << "Tmp_AY:" << std::fixed << std::setprecision(2) << mpu_6500_AY
                        << "\r\n";
                std::cout << "---------------------------"
                        << "\r\n";
                usleep(100000);
            }
            

        }
        break;

        case 'g':
        {
            int fd = pca9685Setup("/dev/i2c-7", 0x70, 400);
            while (true)
            {
                int sp;
                std::cin >> sp;
                pca9685PWMWrite(fd, 16, 0, sp);
            }
        }
        break;

        case 'R':
        {
            long int time;
            long int timee;
            CRSF test(optarg);
            int channelData[50];

            while (true)
            {
                time = GetTimestamp() - TimestartUpLoad;
                //
                int retValue = test.CRSFRead(channelData);
                if (retValue > 0)
                {
                    for (size_t i = 0; i < 15; i++)
                    {
                        std::cout << test.rcToUs(channelData[i]) << " ";
                    }
                    std::cout << "\n";
                }
                else
                {
                    std::cout << "error frame recived"
                              << "\n";
                }
                //
                timee = GetTimestamp() - TimestartUpLoad;
                std::cout << "ret: " << retValue
                          << " last frame time : " << timee - time << " "
                          << "\n";
            }
        } break;

        case 'c':
        {
            SensorsAcorrect();

            int fd = pca9685Setup("/dev/i2c-7", 0x70, 400);
            int pwmValue[4]={1800,1800,1800,1800};
            char input = ' ';

            struct termios old_tio, new_tio;
            tcgetattr(STDIN_FILENO, &old_tio);
            new_tio = old_tio;
            new_tio.c_lflag &=(~ICANON & ~ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

            std::thread inputThread([&input,&pwmValue]{
                while (true) {
                    std::cin >> input;
                if (input == 'w')
                    for(int i=0; i<4; i++) pwmValue[i] += 3;
                else if (input == 's')
                    for(int i=0; i<4; i++) pwmValue[i] -= 3;
                else if (input == 'q')
                    break;
                }
            });

            while (true)
            {
                SensorsParse();

                gyro_roll_input = 0.8 * gyro_roll_input + 0.2 * Angle_Roll;
                pid_roll_error = gyro_roll_input - pid_roll_setpoint;
                pid_i_sum_roll += pid_i_gain_roll * pid_roll_error;
                pid_output_roll = pid_p_gain_roll * pid_roll_error 
                                + pid_i_gain_roll * pid_i_sum_roll 
                                + pid_d_gain_roll * (pid_roll_error - pid_last_d_err);
                pid_last_d_err = pid_roll_error;

                gyro_pitch_input = 0.8 * gyro_pitch_input + 0.2 * Angle_Pitch;
                pid_pitch_error = gyro_pitch_input - pid_pitch_setpoint;
                pid_i_sum_pitch += pid_i_gain_pitch * pid_pitch_error;
                pid_output_pitch = pid_p_gain_pitch * pid_pitch_error 
                                + pid_i_gain_pitch * pid_i_sum_pitch 
                                + pid_d_gain_pitch * (pid_pitch_error - pid_last_d_err_pitch);
                pid_last_d_err_pitch = pid_pitch_error;       

                pca9685PWMWrite(fd, pin1, 0, limitValue(pwmValue[0]));
                pca9685PWMWrite(fd, pin2, 0, limitValue(pwmValue[1]));
                pca9685PWMWrite(fd, pin3, 0, limitValue(pwmValue[2]));
                pca9685PWMWrite(fd, pin4, 0, limitValue(pwmValue[3]));

                std::cout<<"------------------------------------"<< "\r\n";
                std::cout << "Angle_Roll:" << std::fixed << std::setprecision(1) << Angle_Pitch << "\r\n";
                std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(1) << gyro_roll_input << "\r\n";
                std::cout<<"------------------------------------"<< "\r\n";
                std::cout<<"sp = " << pwmValue[0] << "\r\n";
                std::cout<<"sp = " << pwmValue[1] << "\r\n";
                std::cout<<"sp = " << pwmValue[2] << "\r\n";
                std::cout<<"sp = " << pwmValue[3] << "\r\n";
                std::cout<<"------------------------------------"<< "\r\n";

            }
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        }
        break;

    }
    }
}