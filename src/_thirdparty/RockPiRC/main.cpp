#include <fstream>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <sys/time.h>
#include "src/QMC5883/QMC5883.hpp"
#include "src/M10QGPS/M10QGPS.hpp"
#include <csignal>

int TimestartUpLoad = 0;
int signalIn = 0;

inline int GetTimestamp()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}

int main(int argc, char *argv[])
{
    int argvs;
    TimestartUpLoad = GetTimestamp();

    while ((argvs = getopt(argc, argv, "h:s:S:g:G:")) != -1)
    {
        switch (argvs)
        {
        case 'h':
        {
            std::cout << "-g /dev/ttyS0 for GPS\n"
                      << "-G /dev/ttyS0 for GPS Parsed Data\n"
                      << "-S for QML5883 Compass Calibrate\n"
                      << "-s for QML5883 Compass\n";
        }
        break;
        case 'S':
        {
            std::signal(SIGINT, [](int signal)
                        { signalIn = signal; });
            int rawx = 0;
            int rawy = 0;
            int rawz = 0;
            int calibration[10];
            calibration[0] = -5000;
            calibration[1] = 5000;
            calibration[2] = -5000;
            calibration[3] = 5000;
            calibration[4] = -5000;
            calibration[5] = 5000;

            QMC5883 mycompassTest(optarg, 0x0d);
            while (true)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                std::cout << "x:" << std::setw(7) << std::setfill(' ') << rawx << ""
                          << "y:" << std::setw(7) << std::setfill(' ') << rawy << ""
                          << "z:" << std::setw(7) << std::setfill(' ') << rawz << "\r\n";
                mycompassTest.CompassCalibration(true, calibration);
                usleep(50 * 1000);
                if (signalIn == SIGINT)
                    break;
            }
            std::cout << "\r\n";
            std::cout << "XMAX: " << calibration[0] << "\n";
            std::cout << "XMIN: " << calibration[1] << "\n";
            std::cout << "YMAX: " << calibration[2] << "\n";
            std::cout << "YMIN: " << calibration[3] << "\n";
            std::cout << "ZMAX: " << calibration[4] << "\n";
            std::cout << "ZMIN: " << calibration[5] << "\n";
        }
        break;
        case 's':
        {
            int rawx = 0;
            int rawy = 0;
            int rawz = 0;
            double angle = 0;

            QMC5883 mycompassTest(optarg, 0x0d);
            mycompassTest.CompassApply(1010, -2408, 1347, -2110, 1695, -1755);
            while (true)
            {
                mycompassTest.CompassGetRaw(rawx, rawy, rawz);
                mycompassTest.CompassGetUnfixAngle(angle);
                std::cout << "Angle:" << std::setw(7) << std::setfill(' ') << angle << "\r\n";
                std::cout << "x:" << std::setw(7) << std::setfill(' ') << rawx << ""
                          << "y:" << std::setw(7) << std::setfill(' ') << rawy << ""
                          << "z:" << std::setw(7) << std::setfill(' ') << rawz << "\r\n";
                usleep(50 * 1000);
            }
        }
        break;
        case 'G':
        {
            long int time;
            long int timee;
            std::string GPSData;
            M10QGPS GPSUart(optarg);
            GPSUart.GPSReOpen();
            while (true)
            {
                if (GPSUart.GPSCheckDataAvaliable())
                {
                    time = GetTimestamp() - TimestartUpLoad;
                    std::cout << "\nlast frame time get:" << time - timee << "\r\n";
                    GPSUart.GPSRead(GPSData);
                    std::cout << GPSData;
                    timee = GetTimestamp() - TimestartUpLoad;
                }
                usleep(180000);
            }
        }
        break;
        case 'g':
        {
            long int time;
            long int timee;
            std::string GPSData;
            GPSUartData mydata;
            M10QGPS *GPSUart = new M10QGPS(optarg);
            GPSUart->GPSReOpen();
            while (true)
            {
                time = GetTimestamp() - TimestartUpLoad;
                mydata = GPSUart->GPSParse();
                std::cout << "satillites: " << mydata.satillitesCount << " ";
                std::cout << "DataError: " << mydata.DataUnCorrect << " ";
                std::cout << "lat: " << std::setprecision(9) << mydata.lat << " ";
                std::cout << "lng: " << std::setprecision(10) << mydata.lng << " \n";
                std::cout << "ALT: " << std::setprecision(4) << mydata.GPSAlititude << "M "
                          << "HDOP " << std::setprecision(4) << mydata.HDOP << " "
                          << "Quailty: " << mydata.GPSQuality << " "
                          << "GeoidalSP: " << mydata.GPSGeoidalSP << "\n";
                timee = GetTimestamp() - TimestartUpLoad;
                std::cout << "last frame time : " << timee - time << "\n";
                if ((timee - time) > 200000)
                    usleep(1500);
                else
                    usleep(200000 - (timee - time));
            }
        }
        break;
        }
    }
}
