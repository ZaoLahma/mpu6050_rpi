#include "mpu6050.h"

/* C++ */
#include <string_view>
#include <string>
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <iomanip>
#include <algorithm>

/* OS dependencies */
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* External lib dependencies */
extern "C"
{
    #include <linux/i2c-dev.h>
    #include <i2c/smbus.h>
}


namespace mpu6050
{

constexpr std::string_view I2C_DEVICE_FILE {"/dev/i2c-1"};
constexpr uint8_t WORD_LENGTH {16u};

/*
    The cutoff point from which we no longer dare to read data from
    the FIFO buffer in case it wraps over before we can act on the
    overflow interrupt.
*/
constexpr uint16_t FIFO_MAX_CNT {768u};

Mpu6050::Mpu6050(uint8_t i2cAddr) : m_i2cAddr(i2cAddr)
{
    m_i2cFileDescriptor = open(std::string(I2C_DEVICE_FILE).c_str(), O_RDWR);
    if (0 > m_i2cFileDescriptor)
    {
        std::cerr<<"Failed to open the I2C file descriptor at "<<I2C_DEVICE_FILE<<", return code "<<m_i2cFileDescriptor<<"\n";
        abort();
    }

    if (0 != ioctl(m_i2cFileDescriptor, I2C_SLAVE, m_i2cAddr))
    {
        std::cerr<<"Failed to get the I2C bus with address "<<m_i2cAddr<<"\n";
        abort();
    }
}

Mpu6050::~Mpu6050()
{
    close(m_i2cFileDescriptor);
    m_i2cFileDescriptor = -1;
}

bool Mpu6050::wakeup()
{
    return setRegisterValue(MPU6050_REG_PWR_PGMT, MPU6050_REG_SLEEP_MASK, false);
}

bool Mpu6050::sleep()
{
    return setRegisterValue(MPU6050_REG_PWR_PGMT, MPU6050_REG_SLEEP_MASK, true);
}

bool Mpu6050::enableTemperatureSensor()
{
    return setRegisterValue(MPU6050_REG_PWR_PGMT, MPU6050_REG_TEMP_DISABLE_MASK, false);
}

bool Mpu6050::disableTemperatureSensor()
{
    return setRegisterValue(MPU6050_REG_PWR_PGMT, MPU6050_REG_TEMP_DISABLE_MASK, true);
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

bool Mpu6050::setGyroScaleRange250()
{
    return setGyroScaleRange(MPU6050_GYRO_CONFIG_SCALE_RANGE_250);
}

bool Mpu6050::setGyroScaleRange500()
{
    return setGyroScaleRange(MPU6050_GYRO_CONFIG_SCALE_RANGE_500);
}

bool Mpu6050::setGyroScaleRange1000()
{
    return setGyroScaleRange(MPU6050_GYRO_CONFIG_SCALE_RANGE_1000);
}

bool Mpu6050::setGyroScaleRange2000()
{
    return setGyroScaleRange(MPU6050_GYRO_CONFIG_SCALE_RANGE_2000);
}

bool Mpu6050::enableFIFO()
{
    bool retVal {false};

    std::unique_lock<std::mutex> fifoLock(m_fifoThreadMutex);

    if (!m_fifoEnabled)
    {
        m_fifoThread = std::thread(&Mpu6050::fifoReader, this);
    }

    /* Don't return until the FIFO read thread is started */
    m_fifoControlNotification.wait(fifoLock);

    return m_fifoEnabled;
}

bool Mpu6050::disableFIFO()
{
    /* Verify that the FIFO-reading thread stopped... */
    if (m_fifoEnabled)
    {
        m_fifoEnabled = false;
        m_fifoThread.join();
    }

    /* ... and then return the result of the FIFO shutdown (set in FIFO reading thread upon exit) */
    return m_fifoEnabled == false;
}

bool Mpu6050::getFIFOCount(uint16_t& count)
{
    bool retVal {getRegisterWord(MPU6050_REG_FIFO_COUNT_HIGH, MPU6050_REG_FIFO_COUNT_LOW, count)};

    return retVal;
}

void Mpu6050::fifoReader()
{
    /* Enable the data ready interrupt */
    bool retVal {setRegisterValue(MPU6050_REG_INT_ENABLE, MPU6050_REG_DATA_RDY_EN_MASK | MPU6050_FIFO_OFLOW_INT_MASK, true)};

    const uint8_t enableAllSensorsMask {MPU6050_REG_FIFO_TEMP_ENABLE_MASK | MPU6050_REG_FIFO_ACCEL_ENABLE_MASK};

    retVal = retVal && setRegisterValue(MPU6050_REG_FIFO_ENABLE, enableAllSensorsMask, true);

    retVal = retVal && setRegisterValue(MPU6050_REG_USER_CONTROL, MPU6050_REG_FIFO_ENABLE_MASK, true);

    m_fifoEnabled = retVal;

    m_fifoControlNotification.notify_one();

    bool resetFifo {false};
    while (m_fifoEnabled)
    {
        /*
            Empty the FIFO buffer as quickly as we can when the data ready interrupt register is set
            For now handle overflows by simply resetting the FIFO functionality.
        */
        if (uint8_t interrupt {0u};
            resetFifo ||
            (readRegister(MPU6050_REG_INT_STATUS, interrupt) &&
            (MPU6050_FIFO_OFLOW_INT_MASK == (interrupt & MPU6050_FIFO_OFLOW_INT_MASK))))
        {
            if (!setRegisterValue(MPU6050_REG_USER_CONTROL, MPU6050_REG_FIFO_RESET_MASK, true))
            {
                std::cerr<<"Failed to handle FIFO overflow"<<std::endl;
                m_fifoEnabled = false;
            }

            resetFifo = false;
        }
        else if (MPU6050_REG_DATA_RDY_INT_MASK == (MPU6050_REG_DATA_RDY_INT_MASK & interrupt))
        {
            bool failed {false};

            uint16_t fifoCnt {0u};
            while (fifoCnt < FIFO_BUF_SIZE)
            {
                if (!getRegisterWordBurstRead(MPU6050_REG_FIFO_COUNT_HIGH, fifoCnt))
                {
                    failed = true;
                    break;
                }
            }

            if (fifoCnt > FIFO_MAX_CNT)
            {
                /*
                    We're too close to an overflow to want to continue reading.
                    Skip reading and handle as overflow in the next cycle.
                */
                resetFifo = true;
            }
            else if (!failed)
            {
                uint8_t READ_BUF[FIFO_BUF_SIZE];
                if (0 >= getRegisterData(MPU6050_REG_FIFO_READ_WRITE, READ_BUF, FIFO_BUF_SIZE))
                {
                    failed = true;
                }

                if (!failed)
                {
                    std::lock_guard<std::mutex> fifoBufLock(m_fifoThreadMutex);
                    std::copy_n(READ_BUF, FIFO_BUF_SIZE, FIFO_BUF);
                }
            }
        }
    }

    retVal = setRegisterValue(MPU6050_REG_USER_CONTROL, MPU6050_REG_FIFO_ENABLE_MASK, false);

    if (!retVal)
    {
        std::cerr<<"Failed to disable FIFO"<<std::endl;
    }

    retVal = retVal && setRegisterValue(MPU6050_REG_USER_CONTROL, MPU6050_REG_FIFO_RESET_MASK, true);

    if (!retVal)
    {
        std::cerr<<"Failed to reset FIFO buffer"<<std::endl;
    }

    /* If any disable operation failed, keep m_fifoEnabled flag set */
    m_fifoEnabled = !retVal;
}

bool Mpu6050::getTemperature(float& temperature)
{
    bool retVal {false};

    uint16_t registerValue {0x0u};

    if (m_sensorReaderThreadEnabled)
    {
        std::lock_guard<std::mutex> sensorDataLock(m_rawDataThreadMutex);
        registerValue = toUint16(RAW_SENSOR_DATA_BUF, MPU6050_REG_TEMP_HIGH - MPU6050_REG_ACCEL_X_HIGH);
        retVal = true;
    }
    else if (m_fifoEnabled)
    {
        std::lock_guard<std::mutex> fifoBufLock(m_fifoThreadMutex);
        registerValue = toUint16(FIFO_BUF, 6u);
        retVal = true;
    }
    else
    {
        retVal = getRegisterWordBurstRead(MPU6050_REG_TEMP_HIGH, registerValue);
    }

    temperature = static_cast<int16_t>(registerValue) * MPU6050_TEMP_DATA_SCALING_FACTOR;
    temperature = temperature + MPU6050_TEMP_DATA_OFFSET;

    return retVal;
}

bool Mpu6050::getAcceleration(float& x, float& y, float& z)
{
    bool retVal {false};

    uint8_t accelConfig {0x0u};

    retVal = readRegister(MPU6050_REG_ACCEL_CONFIG, accelConfig);

    uint16_t xRead;
    uint16_t yRead;
    uint16_t zRead;
    retVal = retVal && getRegisterWordBurstRead(MPU6050_REG_ACCEL_X_HIGH, xRead);
    retVal = retVal && getRegisterWordBurstRead(MPU6050_REG_ACCEL_Y_HIGH, yRead);
    retVal = retVal && getRegisterWordBurstRead(MPU6050_REG_ACCEL_Z_HIGH, zRead);

    float scalingFactor {0.0f};
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
        retVal = false;
        break;
    }

    float scaledValue = 0.0f;
    scaledValue = static_cast<int16_t>(xRead) * scalingFactor;
    x = scaledValue;
    scaledValue = static_cast<int16_t>(yRead) * scalingFactor;
    y = scaledValue;
    scaledValue = static_cast<int16_t>(zRead) * scalingFactor;
    z = scaledValue;

    return retVal;
}

bool Mpu6050::getGyro(float& x, float& y, float& z)
{
    bool retVal {false};

    uint8_t gyroConfig {0x0u};

    retVal = readRegister(MPU6050_REG_GYRO_CONFIG, gyroConfig);

    uint16_t xRead;
    uint16_t yRead;
    uint16_t zRead;
    retVal = retVal && getRegisterWordBurstRead(MPU6050_REG_GYRO_X_HIGH, xRead);
    retVal = retVal && getRegisterWordBurstRead(MPU6050_REG_GYRO_Y_HIGH, yRead);
    retVal = retVal && getRegisterWordBurstRead(MPU6050_REG_GYRO_Z_HIGH, zRead);

    float scalingFactor {0.0f};

    switch (gyroConfig)
    {
        case MPU6050_GYRO_CONFIG_SCALE_RANGE_250:
        scalingFactor = MPU6050_GYRO_SCALE_250_FACTOR;
        std::cout<<"250"<<std::endl;
        break;
        case MPU6050_GYRO_CONFIG_SCALE_RANGE_500:
        scalingFactor = MPU6050_GYRO_SCALE_500_FACTOR;
        std::cout<<"500"<<std::endl;
        break;
        case MPU6050_GYRO_CONFIG_SCALE_RANGE_1000:
        scalingFactor = MPU6050_GYRO_SCALE_1000_FACTOR;
        std::cout<<"1000"<<std::endl;
        break;
        case MPU6050_GYRO_CONFIG_SCALE_RANGE_2000:
        scalingFactor = MPU6050_GYRO_SCALE_2000_FACTOR;
        std::cout<<"2000"<<std::endl;
        break;
        default:
        retVal = false;
        break;
    }

    std::cout<<"xRead: "<<xRead<<std::endl;
    std::cout<<"yRead: "<<yRead<<std::endl;
    std::cout<<"zRead: "<<zRead<<std::endl;

    float scaledValue = 0.0f;
    scaledValue = static_cast<int16_t>(xRead) * scalingFactor;
    x = scaledValue;
    scaledValue = static_cast<int16_t>(yRead) * scalingFactor;
    y = scaledValue;
    scaledValue = static_cast<int16_t>(zRead) * scalingFactor;
    z = scaledValue;

    return retVal;
}

bool Mpu6050::enableSensorDataThread()
{
    std::unique_lock<std::mutex> sensorLock(m_rawDataThreadMutex);

    if (!m_sensorReaderThreadEnabled)
    {
        m_rawDataThread = std::thread(&Mpu6050::sensorReader, this);
    }

    /* Don't return until the sensor read thread is started */
    m_rawDataControlNotification.wait(sensorLock);

    return m_sensorReaderThreadEnabled;
}

bool Mpu6050::disableSensorDataThread()
{
    if (m_sensorReaderThreadEnabled)
    {
        m_sensorReaderThreadEnabled = false;

        m_rawDataThread.join();
    }

    return true;
}

void Mpu6050::sensorReader()
{
    /* Enable the data ready interrupt */
    bool retVal {setRegisterValue(MPU6050_REG_INT_ENABLE, MPU6050_REG_DATA_RDY_EN_MASK, true)};

    m_sensorReaderThreadEnabled = true;

    m_rawDataControlNotification.notify_one();

    while (m_sensorReaderThreadEnabled)
    {
        if (uint8_t interrupt {0u};
            (readRegister(MPU6050_REG_INT_STATUS, interrupt) &&
            (MPU6050_REG_DATA_RDY_INT_MASK == (interrupt & MPU6050_REG_DATA_RDY_INT_MASK))))
        {
            std::lock_guard<std::mutex> sensorDataLock(m_rawDataThreadMutex);
            bool retVal {getRegisterData(MPU6050_REG_ACCEL_X_HIGH, RAW_SENSOR_DATA_BUF, RAW_SENSOR_DATA_BUF_SIZE) > 0};
        }
    }
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

    uint16_t numReadAttempts {0u};
    constexpr uint8_t MAX_READ_ATTEMPTS {48u};
    while ((numReadAttempts < MAX_READ_ATTEMPTS) && !retVal)
    {
        numReadAttempts++;
        if (int regValue = i2c_smbus_read_byte_data(m_i2cFileDescriptor, mpu6050Register); 0 <= regValue)
        {
            value = regValue;
            retVal = true;
        }   
    }

    if (1u != numReadAttempts)
    {
        std::cout<<"Successful read after "<<numReadAttempts<<std::endl;
    }

    return retVal;
}

bool Mpu6050::setRegisterValue(const uint8_t mpu6050Register, const uint8_t bitmask, const bool set)
{
    bool retVal {false};

    uint8_t regValue {0x0u};

    if (readRegister(mpu6050Register, regValue))
    {
        if (set)
        {
            regValue |= bitmask;
        }
        else
        {
            regValue &= ~bitmask;
        }

        retVal = writeRegister(mpu6050Register, regValue);
    }

    return retVal;
}

bool Mpu6050::getRegisterWord(const uint8_t mpu6050RegisterHigh, const uint8_t mpu6050RegisterLow, uint16_t& value)
{
    bool retVal {true};

    value = 0x0u;

    uint8_t readValue {0x0u};

    retVal = readRegister(mpu6050RegisterHigh, readValue);
    value = (readValue << MPU6050_SENSOR_DATA_OFFSET_HIGH);

    retVal = retVal && readRegister(mpu6050RegisterLow, readValue);
    value = value | (readValue << MPU6050_SENSOR_DATA_OFFSET_LOW);

    return retVal;
}

bool Mpu6050::getRegisterWordBurstRead(const uint8_t mpu6050Register, uint16_t& value)
{
    bool retVal {false};

    value = 0x0u;

    uint8_t buf[WORD_LENGTH];

    if (int32_t readValue {getRegisterData(mpu6050Register, buf, WORD_LENGTH)}; readValue > 0)
    {
        retVal = true;
        value = toUint16(buf, 0);
    }

    return retVal;
}

int32_t Mpu6050::getRegisterData(const uint8_t mpu6050Register, uint8_t buf[], uint8_t bufLength)
{
    int32_t retVal = -1;

    struct i2c_smbus_ioctl_data ioctlData;
    union i2c_smbus_data smbusData;

    smbusData.block[0] = bufLength;

    ioctlData.read_write = I2C_SMBUS_READ;
    ioctlData.command = mpu6050Register;
    ioctlData.size = I2C_SMBUS_I2C_BLOCK_DATA;
    ioctlData.data = &smbusData;

    if (int32_t errorCode = ioctl(m_i2cFileDescriptor, I2C_SMBUS, &ioctlData); 0 == errorCode)
    {
        for(uint32_t i = 0u; i < bufLength; ++i)
        {
            buf[i] = smbusData.block[i + 1u];
        }

        retVal = smbusData.block[0u];
    }

    return retVal;
}

bool Mpu6050::setAccelScaleRange(const uint8_t accelScaleConfig)
{
    return writeRegister(MPU6050_REG_ACCEL_CONFIG, accelScaleConfig);
}

bool Mpu6050::setGyroScaleRange(const uint8_t gyroScaleConfig)
{
    return writeRegister(MPU6050_REG_GYRO_CONFIG, gyroScaleConfig);
}

uint16_t Mpu6050::toUint16(const uint8_t buf[], const uint8_t startPos)
{
    uint16_t value {0u};

    value = buf[startPos] << MPU6050_SENSOR_DATA_OFFSET_HIGH;
    value = value | (buf[startPos + 1u] << MPU6050_SENSOR_DATA_OFFSET_LOW);
    
    return value;
}

} /* Namespace mpu6050 */