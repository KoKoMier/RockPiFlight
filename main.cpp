#include "src/RockPiAPM.hpp"
#include "Drive_Json.hpp"

using namespace RockPiAPMAPI;

#define CONFIGDIR "./APMconfig.json"

void SignalCatch(int Signal);
void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit);
void configWrite(const char *configDir, const char *substr, const char *Target, double obj);

int main(int argc, char *argv[])
{
    int argvs;
    APMSettinngs setting;
    double data[20] = {0};

    while ((argvs = getopt(argc, argv, "r:e:E:a:m")) != -1)
    {
        switch (argvs)
        {
        case 'm':
        {
            std::cout << "./RockPiFlight -r 5inchDefault fot flight start"
                      << "\r\n";
            std::cout << "./RockPiFlight -e 5inchDefault for motor calibration"
                      << "\r\n";
            std::cout << "./RockPiFlight -E 5inchDefault for motor test"
                      << "\r\n";
            std::cout << "./RockPiFlight -a 5inchDefault for MPU6500 Calibrator"
                      << "\r\n";
        }
        break;
        case 'r':
        {
            RockPiAPM APM_Settle;
            configSettle(CONFIGDIR, optarg, setting);
            APM_Settle.RockPiAPMInit(setting);
            //
            std::signal(SIGINT, SignalCatch);
            std::signal(SIGTERM, SignalCatch);
            //
            APM_Settle.RockPiAPMStartUp();
        }
        break;
        case 'e':
        {
            RockPiAPM APM_Settle;
            configSettle(CONFIGDIR, optarg, setting);
            APM_Settle.RockPiAPMInit(setting);
            APM_Settle.APMCalibrator(ESCCalibration, CaliESCStart, 0, data);
        }
        case 'E':
        {
            RockPiAPM APM_Settle;
            configSettle(CONFIGDIR, optarg, setting);
            APM_Settle.RockPiAPMInit(setting);
            while (true)
            {
                int PIN, VALUE;
                std::cin >> PIN;
                std::cin >> VALUE;
                data[0] = PIN;
                APM_Settle.APMCalibrator(ESCCalibration, CaliESCUserDefine, VALUE, data);
            }
        }
        break;
        case 'a':
        {
            int a;
            double tmp[50] = {0};
            RockPiAPM APM_Settle;
            configSettle(CONFIGDIR, optarg, setting);
            APM_Settle.RockPiAPMInit(setting);
            std::cout << "start calibration Nose Up and Type int and enter:"
                      << " \n";
            std::cin >> a;
            APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
            // std::cout << "start calibration Nose Down and Type int and enter:"
            //           << " \n";
            // std::cin >> a;
            // APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
            // std::cout << "start calibration Nose Right Up and Type int and enter:"
            //           << " \n";
            // std::cin >> a;
            // APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
            // std::cout << "start calibration Nose Left Up and Type int and enter:"
            //           << " \n";
            // std::cin >> a;
            // APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
            // std::cout << "start calibration Nose Top  and Type int and enter:"
            //           << " \n";
            // std::cin >> a;
            // APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
            // std::cout << "start calibration Nose Rev and Type int and enter:"
            //           << " \n";
            // std::cin >> a;
            // APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelNoseUp, a, tmp);
            // APM_Settle.APMCalibrator(ACCELCalibration, MPUAccelCaliGet, a, tmp);
            // configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_X_Cali", tmp[MPUAccelCaliX]);
            // configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Y_Cali", tmp[MPUAccelCaliY]);
            // configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Z_Cali", tmp[MPUAccelCaliZ]);
            // configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_X_Scal", tmp[MPUAccelScalX]);
            // configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Y_Scal", tmp[MPUAccelScalY]);
            // configWrite(CONFIGDIR, optarg, "_flag_MPU9250_A_Z_Scal", tmp[MPUAccelScalZ]);
        }
        break;
        }
    }
}

void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Mas = nlohmann::json::parse(content);
    nlohmann::json Configdata = Mas[substr];
    //==========================================================Device Type=======/
    nlohmann::json DC = Configdata["Device"];
    APMInit.DC.__I2CDevice = DC["__I2CDevice"].get<std::string>();
    //==========================================================Controller cofig==/
    nlohmann::json RC = Configdata["Recvier"];
    //==========================================================ESC cofig=========/
    nlohmann::json OC = Configdata["PID"]["Output"];
    APMInit.OC._flag_A1_Pin = OC["_flag_A1_Pin"].get<int>();
    APMInit.OC._flag_A2_Pin = OC["_flag_A2_Pin"].get<int>();
    APMInit.OC._flag_B1_Pin = OC["_flag_B1_Pin"].get<int>();
    APMInit.OC._flag_B2_Pin = OC["_flag_B2_Pin"].get<int>();
    APMInit.OC.ESCPLFrequency = OC["ESCPLFrequency"].get<int>();
    APMInit.OC.ESCControllerType = OC["ESCControllerType"].get<int>();
}

void configWrite(const char *configDir, const char *substr, const char *Target, double obj)
{
    std::ifstream config(configDir);
    std::string content((std::istreambuf_iterator<char>(config)),
                        (std::istreambuf_iterator<char>()));
    nlohmann::json Configdata = nlohmann::json::parse(content);
    nlohmann::json subdata = Configdata[substr]["Sensor"];
    subdata[Target] = obj;
    Configdata[substr]["Sensor"] = subdata;
    std::ofstream configs;
    configs.open(configDir);
    configs << std::setw(4) << Configdata << std::endl;
    configs.close();
}

void SignalCatch(int Signal)
{
    RockPiAPMAPI::SystemSignal = Signal;
};