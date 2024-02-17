#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <sys/time.h>
#include "MPU6500.hpp"
#include <fstream>
#include "Drive_Json.hpp"

int TimestartUpLoad = 0;

double configSettle(const char *configDir, const char *Target)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    return Configdata[Target].get<double>();
}

void configWrite(const char *configDir, const char *Target, double obj)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    Configdata[Target] = obj;
    std::ofstream configs;
    configs.open(configDir);
    configs << std::setw(4) << Configdata << std::endl;
    configs.close();
}

inline int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

int main(int argc, char *argv[])
{
    int argvs;
    int TimeMax = 4000;
    MPUData myData;

    while ((argvs = getopt(argc, argv, "sgm")) != -1)
    {
        switch (argvs)
        {
        case 's':
        {
            std::cout << "Start MPU Calibration\n";
            RPiMPU6500 *myMPUTest = new RPiMPU6500();
            int a;
            double tmp[50];
            std::cout << "start calibration Nose Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseUp, tmp);
            std::cout << "start calibration Nose Down and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseDown, tmp);
            std::cout << "start calibration Nose Right Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseRight, tmp);
            std::cout << "start calibration Nose Left Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseLeft, tmp);
            std::cout << "start calibration Nose Top  and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseTop, tmp);
            std::cout << "start calibration Nose Rev and Type int and enter:"
                      << " \n";
            std::cin >> a;
            myMPUTest->MPUAccelCalibration(MPUAccelNoseRev, tmp);
            myMPUTest->MPUAccelCalibration(MPUAccelCaliGet, tmp);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_X_Cali", tmp[MPUAccelCaliX]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Y_Cali", tmp[MPUAccelCaliY]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Z_Cali", tmp[MPUAccelCaliZ]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_X_Scal", tmp[MPUAccelScalX]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Y_Scal", tmp[MPUAccelScalY]);
            configWrite("./MPUCali.json", "_flag_MPU9250_A_Z_Scal", tmp[MPUAccelScalZ]);
        }
        break;
        case 'm':
        {
            double AccelCaliData[30];
            std::cout << "Calibration Gryo ......";

            RPiMPU6500 *myMPUTest = new RPiMPU6500();
            AccelCaliData[MPUAccelCaliX] = configSettle("./MPUCali.json", "_flag_MPU9250_A_X_Cali");
            AccelCaliData[MPUAccelCaliY] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Y_Cali");
            AccelCaliData[MPUAccelCaliZ] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Z_Cali");
            AccelCaliData[MPUAccelScalX] = configSettle("./MPUCali.json", "_flag_MPU9250_A_X_Scal");
            AccelCaliData[MPUAccelScalY] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Y_Scal");
            AccelCaliData[MPUAccelScalZ] = configSettle("./MPUCali.json", "_flag_MPU9250_A_Z_Scal");
            AccelCaliData[MPUAccelTRIM_Roll] = 0;
            AccelCaliData[MPUAccelTRIMPitch] = 0;
            myMPUTest->MPUCalibration(AccelCaliData);
            std::cout << " Done!\n";

            while (true)
            {
                int microstart = GetTimestamp();
                myData = myMPUTest->MPUSensorsDataGet();
                std::cout << "\033[20A";
                std::cout << "\033[K";
                std::cout << "Accel Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel__Roll << "|"
                          << "AccelPitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Accel_Pitch << "| \n";
                std::cout << "ACC       X: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU6500_A_X << "|"
                          << "ACC       Y: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU6500_A_Y << "|"
                          << "ACC       Z: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_MPU6500_A_Z << "| \n";
                std::cout << "Gryo  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo__Roll << "|"
                          << "Gryo Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo_Pitch << "|"
                          << "Gryo   Yaw: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Gryo___Yaw << "| \n";
                std::cout << "Real  Roll: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real__Roll << "|"
                          << "Real Pitch: " << std::setw(7) << std::setfill(' ') << (int)myData._uORB_Real_Pitch << "| \n";
                int microend = GetTimestamp();
                if (microend - microstart < TimeMax)
                {
                    usleep(TimeMax - (microend - microstart)); // 250Hz
                }
            }
        }
        break;
        }
    }
}
