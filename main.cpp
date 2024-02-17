#include "src/RockPiAPM.hpp"

using namespace RockPiAPMAPI;

#define CONFIGDIR "/etc/APMconfig.json"

int TimestartUpLoad = 0;
void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit);

int main(int argc, char *argv[])
{
    int argvs;
    APMSettinngs setting;

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

        }
        break;
        }
    }
}

void configSettle(const char *configDir, const char *substr, APMSettinngs &APMInit)
{
}