#include "mpu6050.h"

/* C++ */
#include <string_view>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <cmath> 

/* OS dependencies */
#include <fcntl.h>
#include <sys/ioctl.h>

/* External dependencies */
extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}


namespace mpu6050
{

constexpr std::string_view I2C_DEVICE_FILE {"/dev/i2c-1"};

Mpu6050::Mpu6050(uint8_t i2cAddr) : m_i2cAddr(i2cAddr)
{
    m_i2cFileDescriptor = open(std::string(I2C_DEVICE_FILE).c_str(), O_RDWR);
    if (0 > m_i2cFileDescriptor)
    {
        std::cout<<"Failed to open the I2C file descriptor at "<<I2C_DEVICE_FILE<<", return code "<<m_i2cFileDescriptor<<"\n";
        abort();
    }

    if (0 != ioctl(m_i2cFileDescriptor, I2C_SLAVE, m_i2cAddr))
    {
        std::cout<<"Failed to get the I2C bus with address "<<m_i2cAddr<<"\n";
        abort();
    }
}

bool Mpu6050::wakeup()
{
    bool retVal {false};
    uint8_t regValue {0x0u};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValue))
    {
        regValue &= ~MPU6050_REG_SLEEP_MASK;
        if (writeRegister(MPU6050_REG_PWR_PGMT, regValue))
        {
            retVal = true;
        }
    }

    return retVal;
}

bool Mpu6050::sleep()
{
    bool retVal {false};
    uint8_t regValue {0x0u};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValue))
    {
        regValue |= MPU6050_REG_SLEEP_MASK;
        if (writeRegister(MPU6050_REG_PWR_PGMT, regValue))
        {
            retVal = true;
        }
    }

    return retVal;
}

bool Mpu6050::enableTemperatureSensor()
{
    bool retVal {false};
    uint8_t regValue {0x0u};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValue))
    {
        regValue &= ~MPU6050_REG_TEMP_DISABLE_MASK;
        if (writeRegister(MPU6050_REG_PWR_PGMT, regValue))
        {
            retVal = true;
        }
    }

    return retVal;
}

bool Mpu6050::disableTemperatureSensor()
{
    bool retVal {false};
    uint8_t regValue {0x0u};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValue))
    {
        regValue |= MPU6050_REG_TEMP_DISABLE_MASK;
        if (writeRegister(MPU6050_REG_PWR_PGMT, regValue))
        {
            retVal = true;
        }
    }

    return retVal;
}

bool Mpu6050::resetRegisters()
{
    return writeRegister(MPU6050_REG_PWR_PGMT, MPU6050_REG_RESET_MASK);
}

bool Mpu6050::setAccelScaleRange2G()
{
    return setAccelScaleRange(MPU6050_ACCEL_CONFIG_SCALE_RANGE_2G);
}

bool Mpu6050::setAccelScaleRange4G()
{
    return setAccelScaleRange(MPU6050_ACCEL_CONFIG_SCALE_RANGE_4G);
}

bool Mpu6050::setAccelScaleRange8G()
{
    return setAccelScaleRange(MPU6050_ACCEL_CONFIG_SCALE_RANGE_8G);
}

bool Mpu6050::setAccelScaleRange16G()
{
    return setAccelScaleRange(MPU6050_ACCEL_CONFIG_SCALE_RANGE_16G);
}

bool Mpu6050::getTemperature(float& temperature)
{
    bool retVal {false};

    int16_t registerValue = 0.0f;
    retVal = getSensorData(MPU6050_REG_TEMP_HIGH, MPU6050_REG_TEMP_LOW, registerValue);

    temperature = registerValue * MPU6050_TEMP_DATA_SCALING_FACTOR;
    temperature = temperature + MPU6050_TEMP_DATA_OFFSET;

    return retVal;
}

bool Mpu6050::getAcceleration(float& x, float& y, float& z)
{
    bool retVal {false};

    uint8_t accelConfig {0x0u};

    retVal = readRegister(MPU6050_REG_ACCEL_CONFIG, accelConfig);

    int16_t xRead;
    int16_t yRead;
    int16_t zRead;
    retVal = retVal && getSensorData(MPU6050_REG_ACCEL_X_HIGH, MPU6050_REG_ACCEL_X_LOW, xRead);
    retVal = retVal && getSensorData(MPU6050_REG_ACCEL_Y_HIGH, MPU6050_REG_ACCEL_Y_LOW, yRead);
    retVal = retVal && getSensorData(MPU6050_REG_ACCEL_Z_HIGH, MPU6050_REG_ACCEL_Z_LOW, zRead);

    float scalingFactor {0.0f};

    /* TODO: Fix the switch below */
    switch (accelConfig)
    {
        case MPU6050_ACCEL_CONFIG_SCALE_RANGE_2G:
        scalingFactor = MPU6050_ACCEL_SCALE_2G_FACTOR;
        break;
        case MPU6050_ACCEL_CONFIG_SCALE_RANGE_4G:
        scalingFactor = MPU6050_ACCEL_SCALE_4G_FACTOR;
        break;
        case MPU6050_ACCEL_CONFIG_SCALE_RANGE_8G:
        scalingFactor = MPU6050_ACCEL_SCALE_8G_FACTOR;
        break;
        case MPU6050_ACCEL_CONFIG_SCALE_RANGE_16G:
        scalingFactor = MPU6050_ACCEL_SCALE_16G_FACTOR;
        break;
        default:
        break;
    }

    float scaledValue = 0.0f;
    scaledValue = xRead * scalingFactor;
    x = scaledValue;
    scaledValue = yRead * scalingFactor;
    y = scaledValue;
    scaledValue = zRead * scalingFactor;
    z = scaledValue;

    return retVal;
}

bool Mpu6050::writeRegister(const uint8_t mpu6050Register, const uint8_t value)
{
    bool retVal {false};

    if (0 == i2c_smbus_write_byte_data(m_i2cFileDescriptor, mpu6050Register, value))
    {
        retVal = true;
    }

    return retVal;
}

bool Mpu6050::readRegister(const uint8_t mpu6050Register, uint8_t& value)
{
    bool retVal {false};

    if (int regValue = i2c_smbus_read_byte_data(m_i2cFileDescriptor, mpu6050Register); 0 <= regValue)
    {
        value = regValue;
        retVal = true;
    }

    return retVal;
}

bool Mpu6050::getSensorData(uint8_t mpu6050RegisterHigh, uint8_t mpu6050RegisterLow, int16_t& value)
{
    bool retVal {true};

    value = 0x0u;

    uint8_t readValue = 0x0u;

    retVal = readRegister(mpu6050RegisterHigh, readValue);
    value = (readValue << MPU6050_SENSOR_DATA_OFFSET_HIGH);

    retVal = retVal && readRegister(mpu6050RegisterLow, readValue);
    value = value | (readValue << MPU6050_SENSOR_DATA_OFFSET_LOW);

    return retVal;
}

bool Mpu6050::setAccelScaleRange(const uint8_t accelScaleConfig)
{
    return writeRegister(MPU6050_REG_ACCEL_CONFIG, accelScaleConfig);
}

} /* Namespace mpu6050 */