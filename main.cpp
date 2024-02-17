#include "src/RockPiAPM.hpp"
#include "Drive_Json.hpp"

using namespace RockPiAPMAPI;

#define CONFIGDIR "./APMconfig.json"

int TimestartUpLoad = 0;
void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit);

int main(int argc, char *argv[])
{
    int argvs;
    APMSettinngs setting;
    double data[20] = {0};

    while ((argvs = getopt(argc, argv, "e:E:m")) != -1)
    {
        switch (argvs)
        {
        case 'm':
        {
            std::cout << "./RockPiFlight -e 5inchDefault for motor calibration"
                      << "\r\n";
            std::cout << "./RockPiFlight -E 5inchDefault for motor test"
                      << "\r\n";
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