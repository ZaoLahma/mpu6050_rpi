#include "mpu6050.h"

#include <string_view>
#include <string>
#include <iostream>
#include <stdlib.h> 

#include <fcntl.h>
#include <sys/ioctl.h>

extern "C" {
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
    int regValueInt {0};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValueInt))
    {
        uint8_t regValue = regValueInt;
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
    int regValueInt {0};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValueInt))
    {
        uint8_t regValue = regValueInt;
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
    int regValueInt {0};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValueInt))
    {
        uint8_t regValue = regValueInt;
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
    int regValueInt {0};
    if (true == readRegister(MPU6050_REG_PWR_PGMT, regValueInt))
    {
        uint8_t regValue = regValueInt;
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

bool Mpu6050::writeRegister(const uint8_t mpu6050Register, const uint8_t value)
{
    bool retVal {false};

    if (0 == i2c_smbus_write_byte_data(m_i2cFileDescriptor, mpu6050Register, value))
    {
        std::cout<<"Wrote "<<static_cast<uint32_t>(value)<<" to "<<static_cast<uint32_t>(mpu6050Register)<<std::endl;
        retVal = true;
    }

    return retVal;
}

bool Mpu6050::readRegister(const uint8_t mpu6050Register, int& value)
{
    bool retVal {false};

    if (int regValue = i2c_smbus_read_byte_data(m_i2cFileDescriptor, mpu6050Register); 0 <= regValue)
    {
        value = regValue;
        retVal = true;
    }

    return retVal;
}

} /* Namespace mpu6050 */