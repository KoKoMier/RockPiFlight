#pragma once
#include <stdint.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <iostream>

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define PIN_ALL 16
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA

inline uint16_t baseReg(int pin)
{
    return (pin >= PIN_ALL ? LEDALL_ON_L : LED0_ON_L + 4 * pin);
}

inline int pca9685PWMFreq(int fd, float frequency)
{
    frequency = (frequency > 1526 ? 1526 : (frequency < 24 ? 24 : frequency));
    int prescale = (int)(25000000.0f / (4096 * frequency) - 0.5);
    // read
    uint8_t data[8] = {0x00};
    uint8_t reg = PCA9685_MODE1;
    // if()
    return 0;
}

inline int pca9685PWMSetup(const char *I2CChannel, uint8_t I2CAddress, int frequency)
{
    int fd;
    if ((fd = open(I2CChannel, O_RDWR)) < 0)
        return -1;
    if (ioctl(fd, I2C_SLAVE, I2CAddress) < 0)
        return -2;
    if (ioctl(fd, I2C_TIMEOUT, 0x01) < 0) // set to 10ms?
        return -2;

    uint8_t data[8] = {0x00};
    // Read
    uint8_t reg = PCA9685_MODE1;
    if (write(fd, &reg, 1) < 0)
        return -3;
    if (read(fd, data, 1) < 0)
        return -4;
    std::cout << "data" << data << "\r\n";
    return 0;
}

// inline int pca9685PWMReset(int fd)
// {
// }