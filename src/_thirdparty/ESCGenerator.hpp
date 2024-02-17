#pragma once
#include "PCA9685/pca9685.hpp"

#define PCA9685_RANGE 4096.f
#define PCA9685_ALL_PIN 16
#define ONESHOT125_RANGE_LOW 125.f
#define ONESHOT125_RANGE_HIGH 250.f
#define PWM_MAX_US 2000.f
#define PWM_MIN_US 1000.f
#define PWM_RANGE 1000.f

enum GeneratorType
{
    Hardware_PWM,
    Software_PWM,
    Hardware_ONESHOT125,
    Software_ONESHOT125,
};

class ESCGenerator
{
public:
    inline ESCGenerator(GeneratorType generator, const char *device, uint8_t addr, int Frequence);

    inline void ESCUpdate(int ID, int Range);

    inline void ESCClear(int ID);

    inline ~ESCGenerator()
    {
        pca9685PWMReset(GeneratorFD);
        pca9685PWMResetON(GeneratorFD, PCA9685_ALL_PIN);
        close(GeneratorFD);
    }

private:
    GeneratorType Generator;
    int PlFrequency = 0;
    int GeneratorFD = -1;
    int ThrotteMin;
    int ThrotteMax;
    int ThrotteRange;
};

ESCGenerator::ESCGenerator(GeneratorType generator, const char *device, uint8_t addr, int Frequence)
{
    Generator = generator;
    PlFrequency = Frequence;
    switch (Generator)
    {
    case GeneratorType::Hardware_PWM:
    {
        GeneratorFD = pca9685Setup(device, addr, PlFrequency);
        pca9685PWMReset(GeneratorFD);
        pca9685PWMResetON(GeneratorFD, PCA9685_ALL_PIN);
        ThrotteMin = (PCA9685_RANGE - 1) / 2.f;
        ThrotteMax = (PCA9685_RANGE - 1);
        ThrotteRange = ThrotteMax - ThrotteMin;
    }
    break;
    case GeneratorType::Hardware_ONESHOT125:
    {
        GeneratorFD = pca9685Setup(device, addr, PlFrequency);
        pca9685PWMReset(GeneratorFD);
        pca9685PWMResetON(GeneratorFD, PCA9685_ALL_PIN);
        int dt = 1.f / (float)PlFrequency * 1000000.f;
        ThrotteMin = PCA9685_RANGE * (ONESHOT125_RANGE_LOW / dt);
        ThrotteMax = PCA9685_RANGE * (ONESHOT125_RANGE_HIGH / dt);
        ThrotteRange = ThrotteMax - ThrotteMin;
    }
    break;
    case GeneratorType::Software_PWM:
        throw 0;
        break;
    case GeneratorType::Software_ONESHOT125:
        throw 0;
        break;
    default:
        throw 0;
        break;
    }
    if (GeneratorFD < 0)
        throw std::invalid_argument("[ESC] I2C process failed");
}

inline void ESCGenerator::ESCUpdate(int ID, int Range)
{
    int Output = ThrotteMin + ((((float)Range - PWM_MIN_US) / PWM_RANGE) * (float)ThrotteRange);
    switch (Generator)
    {
    case GeneratorType::Hardware_PWM:
        pca9685PWMWriteS(GeneratorFD, ID, Output);
        break;

    case GeneratorType::Hardware_ONESHOT125:
        pca9685PWMWriteS(GeneratorFD, ID, Output);
        break;

    case GeneratorType::Software_PWM:
        throw 0;
        break;

    case GeneratorType::Software_ONESHOT125:
        throw 0;
        break;

    default:
        throw 0;
        break;
    }
};

inline void ESCGenerator::ESCClear(int ID)
{
    switch (Generator)
    {
    case GeneratorType::Hardware_PWM:
        pca9685PWMWrite(GeneratorFD, ID, 0, 0);
        break;

    case GeneratorType::Hardware_ONESHOT125:
        pca9685PWMWrite(GeneratorFD, ID, 0, 0);
        break;

    case GeneratorType::Software_PWM:
        throw 0;
        break;

    case GeneratorType::Software_ONESHOT125:
        throw 0;
        break;

    default:
        throw 0;
        break;
    }
}