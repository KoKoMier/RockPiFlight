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

int signalIn = 0;
int TimestartUpLoad = 0;

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
            int fd = pca9685Setup("/dev/i2c-7", 0x70, 400);
            int sp=1800;
            char input;

            struct termios old_tio, new_tio;
            tcgetattr(STDIN_FILENO, &old_tio);
            new_tio = old_tio;
            new_tio.c_lflag &=(~ICANON & ~ECHO);
            tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);

            while (true)
            {
                std::cin >> input;
                if (input == 'w')
                    sp += 10;
                else if (input == 's')
                    sp -= 10;
                else if (input == 'q')
                    break;
                pca9685PWMWrite(fd, 16, 0, sp);
                std::cout<<"sp = " << sp << "\r\n";
                std::cout<<"777";
            }
            tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
        }
        break;

    }
    }
}