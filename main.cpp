#include <iostream>
#include "src/MPU6500/MPU6500.hpp"
#include "src/spi/LinuxSPI.hpp"
#include <chrono>
#include <unistd.h>
#include <iomanip>
#include <cmath>
#include "src/PAC9685/pac.hpp"
#include "src/UART/CRSFProtocol.hpp"
#include "src/UART/CRSFUartRC.hpp"
#include <sys/time.h>
#include <thread>
#include "src/SYS/sys.hpp"
#include "src/control/control.hpp"


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

int pwmValue[4] = {1800, 1800, 1800, 1800};
uint8_t imu_get_data_flag = 0;
int imu_no_get_data_count = 0;

int pin1 = 15;
int pin2 = 13;
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

float limitValue(float value)
{
    if (value > 3200)
    {
        return 3200;
    }
    else
    {
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
            CONTROL control;
            SensorsAcorrect();

            while (true)
            {
                int microstart = GetTimestamp();
                //control.control();
                SensorsParse();
                //
                //std::cout << "---------------------------"
                //          << "\r\n";
                // std::cout << "mpu_6500_GX:" << std::fixed << std::setprecision(5) << mpu_6500_GX << "\r\n";
                // std::cout << "mpu_6500_GY:" << std::fixed << std::setprecision(5) << mpu_6500_GY << "\r\n";
                // std::cout << "mpu_6500_GZ:" << std::fixed << std::setprecision(5) << mpu_6500_GZ << "\r\n";
                // std::cout << "---------------------------"
                //         << "\r\n";
                //std::cout << "Angle_Pitch_Out:" << std::fixed << std::setprecision(1) << Angle_Pitch_Out << "\r\n";
                //std::cout << "Angle_Roll_Out:" << std::fixed << std::setprecision(1) << Angle_Roll_Out << "\r\n";
                // std::cout << "Tmp_AY:" << std::fixed << std::setprecision(2) << mpu_6500_AY
                //         << "\r\n";
                //std::cout << "---------------------------"
                //          << "\r\n";
                int microend = GetTimestamp();
                if(microend - microstart < 4000)
                {
                    usleep(4000 - (microend - microstart)); // 250Hz
                }
            }
        }
        break;

        case 'f':
        {
            while (true)
            {
                FILE *file = fopen("../data/example.txt", "w");
                if (file == NULL) {
                    printf("无法打开文件\n");
                    return 1;
                }

                for (int i = 1; i <= 6; i++) {
                    int sp;
                    switch (i)
                    {
                        case 1:
                            {
                                std::cout<<"请将无人机放平"<<"\r\n";
                                std::cin >> sp;
                                SensorsAcorrect();
                                std::cout<<"_mpu_6500_AZ_Cali:"<<_mpu_6500_AZ_Cali<<"\r\n";
                                fprintf(file, "Orthogonal data:%f\n", _mpu_6500_AZ_Cali);
                            } break;
                        case 2:
                            {
                                std::cout<<"请将无人机倒放"<<"\r\n";
                                std::cin >> sp;
                                SensorsAcorrect();
                                std::cout<<"_mpu_6500_AZ_Cali:"<<_mpu_6500_AZ_Cali<<"\r\n";
                                fprintf(file, "Inverted data:%f\n", _mpu_6500_AZ_Cali);
                            } break;
                        case 3:
                            {
                                std::cout<<"请将无人机左侧放"<<"\r\n";
                                std::cin >> sp;
                                SensorsAcorrect();
                                std::cout<<"_mpu_6500_AX_Cali:"<<_mpu_6500_AX_Cali<<"\r\n";                                
                                fprintf(file, "Left data:%f\n", _mpu_6500_AX_Cali);
                            } break;
                        case 4:
                            {
                                std::cout<<"请将无人机右侧放"<<"\r\n";
                                std::cin >> sp;
                                SensorsAcorrect();
                                std::cout<<"_mpu_6500_AX_Cali:"<<_mpu_6500_AX_Cali<<"\r\n";   
                                fprintf(file, "Right data:%f\n", _mpu_6500_AX_Cali);
                            } break;
                        case 5:
                            {
                                std::cout<<"请将无人机前侧放"<<"\r\n";
                                std::cin >> sp;
                                SensorsAcorrect();
                                std::cout<<"_mpu_6500_AX_Cali:"<<_mpu_6500_AY_Cali<<"\r\n";   
                                fprintf(file, "Front data:%f\n", _mpu_6500_AY_Cali);
                            } break;
                        case 6:
                            {
                                std::cout<<"请将无人机后侧放"<<"\r\n";
                                std::cin >> sp;
                                SensorsAcorrect();
                                std::cout<<"_mpu_6500_AX_Cali:"<<_mpu_6500_AY_Cali<<"\r\n";   
                                fprintf(file, "Back  data:%f\n", _mpu_6500_AY_Cali);
                            } break;
                    }
                }
                std::cout<<"初始化完毕"<<"\r\n";
                fclose(file);

                return 0;
            }

        }
        break;

        case 'g':
        {
            int fd = pca9685Setup("/dev/i2c-7", 0x40, 400);
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
            CRSF test;
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
        }
        break;

        case 'c':
        {
            SensorsAcorrect();
            

            CONTROL control;
            int fd = pca9685Setup("/dev/i2c-7", 0x70, 400);
            char input = ' ';

            struct termios old_tio, new_tio;
            tcgetattr(STDIN_FILENO, &old_tio);
            new_tio = old_tio;
            new_tio.c_lflag &= (~ICANON & ~ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

            std::thread inputThread([&input, &control]
                                    {
                static int throttle = 1800;
                static double roll_target_angle = 0.65;

                while (true) {
                    std::cin >> input;
                if (input == 'w')
                {
                    throttle += 3;
                    control.set_throttle(throttle);
                }
                else if (input == 's')
                {
                    throttle -= 3;
                    control.set_throttle(throttle);
                }
                else if (input == 'a')
                {
                    roll_target_angle += 0.05;
                    control.set_target_angle(-2.0, roll_target_angle, 0);
                }
                else if (input == 'd')
                {
                    roll_target_angle -= 0.05;
                    control.set_target_angle(-2.0, roll_target_angle, 0);
                }
                else if (input == 'q')
                {
                    pwmValue[0] = 1800;
                    pwmValue[1] = 1800;
                    pwmValue[2] = 1800;
                    pwmValue[3] = 1800;
                    control.set_throttle(1800);
                }


                } });

            std::thread inputThread2([&fd, &control]
                                    {
                while (true){
                    int microstart = GetTimestamp();
                    control.control();
                    pca9685PWMWriteS(fd, pin1, limitValue(pwmValue[0]));
                    pca9685PWMWriteS(fd, pin2, limitValue(pwmValue[1]));
                    pca9685PWMWriteS(fd, pin3, limitValue(pwmValue[2]));
                    pca9685PWMWriteS(fd, pin4, limitValue(pwmValue[3]));
                     int microend = GetTimestamp();
                std::cout << "time = " << microend - microstart << "\r\n";
                } });

            while (true)
            {
                int microstart = GetTimestamp();
                SensorsParse();

                // gyro_roll_input = 0.8 * gyro_roll_input + 0.2 * Angle_Roll;
                // pid_roll_error = gyro_roll_input - pid_roll_setpoint;
                // pid_i_sum_roll += pid_i_gain_roll * pid_roll_error;
                // pid_output_roll = pid_p_gain_roll * pid_roll_error + pid_i_gain_roll * pid_i_sum_roll + pid_d_gain_roll * (pid_roll_error - pid_last_d_err);
                // pid_last_d_err = pid_roll_error;

                // gyro_pitch_input = 0.8 * gyro_pitch_input + 0.2 * Angle_Pitch;
                // pid_pitch_error = gyro_pitch_input - pid_pitch_setpoint;
                // pid_i_sum_pitch += pid_i_gain_pitch * pid_pitch_error;
                // pid_output_pitch = pid_p_gain_pitch * pid_pitch_error + pid_i_gain_pitch * pid_i_sum_pitch + pid_d_gain_pitch * (pid_pitch_error - pid_last_d_err_pitch);
                // pid_last_d_err_pitch = pid_pitch_error;

                // std::cout << "pwm1: " << pwmValue[0] << std::endl;

                // std::cout << "------------------------------------"
                //           << "\r\n";
                // std::cout << "Angle_Pitch:" << std::fixed << std::setprecision(1) << Angle_Pitch_Out << "\r\n";
                // std::cout << "Angle_Roll:" << std::fixed << std::setprecision(1) << Angle_Roll_Out << "\r\n";
                // std::cout << "------------------------------------"
                //           << "\r\n";
                // std::cout << "sp = " << pwmValue[0] << "\r\n";
                // std::cout << "sp = " << pwmValue[1] << "\r\n";
                // std::cout << "sp = " << pwmValue[2] << "\r\n";
                // std::cout << "sp = " << pwmValue[3] << "\r\n";
                // std::cout << "------------------------------------"
                        //   << "\r\n";

                int microend = GetTimestamp();
                // std::cout << "time = " << microend - microstart << "\r\n";
                if(microend - microstart < 4000)
                {
                    usleep(4000 - (microend - microstart)); // 250Hz
                }
            }
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        }
        break;
        }
    }
}