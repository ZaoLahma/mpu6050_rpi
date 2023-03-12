#pragma once

#include <cstdint>

namespace mpu6050
{

/* Default device I2C address */
constexpr uint8_t MPU6050_DEFAULT_I2C_ADDR {0x68};

/* Register bit masks */
constexpr uint8_t MPU6050_REG_SLEEP_MASK               {1u << 6u};
constexpr uint8_t MPU6050_REG_RESET_MASK               {1u << 7u};
constexpr uint8_t MPU6050_REG_TEMP_DISABLE_MASK        {1u << 4u};

/* Registers */
constexpr uint8_t MPU6050_REG_PWR_PGMT                 {0x6Bu};
constexpr uint8_t MPU6050_REG_TEMP_HIGH                {0x41u};
constexpr uint8_t MPU6050_REG_TEMP_LOW                 {0x42u};

/* Bit offsets */
constexpr uint8_t MPU6050_SENSOR_DATA_OFFSET_HIGH      {8u};
constexpr uint8_t MPU6050_SENSOR_DATA_OFFSET_LOW       {0u};

/* Sensor data conversion */
constexpr float MPU6050_TEMP_DATA_SCALING_FACTOR       {1.0f / 340.0f};
constexpr float MPU6050_TEMP_DATA_OFFSET               {36.53f};

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

    /* Sensor data */
    bool getTemperature(int16_t& temperature);

    /* General register write */
    bool writeRegister(const uint8_t mpu6050Register, const uint8_t value);

    /* General register read */
    bool readRegister(const uint8_t mpu6050Register, uint8_t& value);

    protected:

    private:
    uint8_t m_i2cAddr;
    int m_i2cFileDescriptor;

    bool getSensorData(uint8_t mpu6050RegisterHigh, uint8_t mpu6050RegisterLow, int16_t& value);
};

} /* namespace mpu6050 */