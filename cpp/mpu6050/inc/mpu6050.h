#pragma once

#include <cstdint>

namespace mpu6050
{

/* Default device I2C address */
constexpr uint8_t MPU6050_DEFAULT_I2C_ADDR {0x68};

/* Register bit masks */
constexpr uint8_t MPU6050_REG_SLEEP_MASK        {1u << 6u};
constexpr uint8_t MPU6050_REG_RESET_MASK        {1u << 7u};
constexpr uint8_t MPU6050_REG_TEMP_DISABLE_MASK {1u << 4u};

/* Registers */
constexpr uint8_t MPU6050_REG_PWR_PGMT {0x6B};

class Mpu6050
{
    public:
    Mpu6050(uint8_t i2cAddr = MPU6050_DEFAULT_I2C_ADDR);

    /* Power and device state management */
    bool wakeup();
    bool sleep();
    bool enableTemperatureSensor();
    bool disableTemperatureSensor();
    bool resetRegisters();

    /* General register write */
    bool writeRegister(const uint8_t mpu6050Register, const uint8_t value);

    /* General register read */
    bool readRegister(const uint8_t mpu6050Register, int& value);

    protected:

    private:
    uint8_t m_i2cAddr;
    int m_i2cFileDescriptor; 
};

} /* namespace mpu6050 */