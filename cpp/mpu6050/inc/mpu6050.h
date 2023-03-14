#pragma once

#include <cstdint>

namespace mpu6050
{

/* Default device I2C address */
constexpr uint8_t MPU6050_DEFAULT_I2C_ADDR             {0x68};

/* Register bit masks */
constexpr uint8_t MPU6050_REG_FIFO_ENABLE_MASK         {1u << 6u};
constexpr uint8_t MPU6050_REG_FIFO_RESET_MASK          {1u << 2u};

constexpr uint8_t MPU6050_REG_SLEEP_MASK               {1u << 6u};
constexpr uint8_t MPU6050_REG_RESET_MASK               {1u << 7u};
constexpr uint8_t MPU6050_REG_TEMP_DISABLE_MASK        {1u << 4u};

constexpr uint8_t MPU6050_REG_FIFO_TEMP_ENABLE_MASK    {1u << 7u};
constexpr uint8_t MPU6050_REG_FIFO_ACCEL_ENABLE_MASK   {1u << 3u};

/* Registers */
constexpr uint8_t MPU6050_REG_USER_CONTROL             {0x6A};

constexpr uint8_t MPU6050_REG_FIFO_ENABLE              {0x23u};
constexpr uint8_t MPU6050_REG_FIFO_COUNT_HIGH          {0x72u};
constexpr uint8_t MPU6050_REG_FIFO_COUNT_LOW           {0x73u};
constexpr uint8_t MPU6050_REG_FIRO_READ_WRITE          {0x74u};

constexpr uint8_t MPU6050_REG_PWR_PGMT                 {0x6Bu};

constexpr uint8_t MPU6050_REG_TEMP_HIGH                {0x41u};
constexpr uint8_t MPU6050_REG_TEMP_LOW                 {0x42u};

constexpr uint8_t MPU6050_REG_ACCEL_CONFIG             {0x1Cu};
constexpr uint8_t MPU6050_REG_ACCEL_X_HIGH             {0x3Bu};
constexpr uint8_t MPU6050_REG_ACCEL_X_LOW              {0x3Cu};
constexpr uint8_t MPU6050_REG_ACCEL_Y_HIGH             {0x3Du};
constexpr uint8_t MPU6050_REG_ACCEL_Y_LOW              {0x3Eu};
constexpr uint8_t MPU6050_REG_ACCEL_Z_HIGH             {0x3Fu};
constexpr uint8_t MPU6050_REG_ACCEL_Z_LOW              {0x40u};

/* Sensor data bit offsets */
constexpr uint8_t MPU6050_SENSOR_DATA_OFFSET_HIGH      {8u};
constexpr uint8_t MPU6050_SENSOR_DATA_OFFSET_LOW       {0u};

/* Sensor data conversion */
constexpr float MPU6050_TEMP_DATA_SCALING_FACTOR       {1.0f / 340.0f};
constexpr float MPU6050_TEMP_DATA_OFFSET               {36.53f};

constexpr float MPU6050_ACCEL_SCALE_2G_FACTOR          {1.0f / 16384.0f};
constexpr float MPU6050_ACCEL_SCALE_4G_FACTOR          {1.0f / 8192.0f};
constexpr float MPU6050_ACCEL_SCALE_8G_FACTOR          {1.0f / 4096.0f};
constexpr float MPU6050_ACCEL_SCALE_16G_FACTOR         {1.0f / 2048.0f};

/* Sensor configuration */
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_2G  {0u << 3u};
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_4G  {1u << 3u};
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_8G  {2u << 3u};
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_16G {3u << 3u};

class Mpu6050
{
    public:
    Mpu6050(uint8_t i2cAddr = MPU6050_DEFAULT_I2C_ADDR);
    ~Mpu6050();

    /* Power and device state management */
    bool wakeup();
    bool sleep();
    bool enableTemperatureSensor();
    bool disableTemperatureSensor();
    bool resetRegisters();

    /* FIFO state management */
    bool enableFIFO();
    bool getFIFOSampleCount(uint16_t& count);
    bool enableTemperatureSensorFIFO();
    bool enableAccelSensorFIFO();

    /*
       FIFO data, in sensor register address order.
       At most 32 bytes, so supports at most 16 enabled
       FIFO sensors at one time
    */
    bool readFIFO(uint16_t values[], uint8_t numValues);

    /* Sensor configuration */
    bool setAccelScaleRange2G();
    bool setAccelScaleRange4G();
    bool setAccelScaleRange8G();
    bool setAccelScaleRange16G();

    /* Sensor data */
    bool getTemperature(float& temperature);
    bool getAcceleration(float& x, float& y, float& z);

    /* General register write */
    bool writeRegister(const uint8_t mpu6050Register, const uint8_t value);

    /* General register read */
    bool readRegister(const uint8_t mpu6050Register, uint8_t& value);

    protected:

    private:
    uint8_t m_i2cAddr;
    int m_i2cFileDescriptor;

    bool setRegisterValue(const uint8_t mpu6050Register, const uint8_t bitmask, const bool set);

    bool getRegisterWord(const uint8_t mpu6050RegisterHigh, const uint8_t mpu6050RegisterLow, uint16_t& value);

    int getRegisterData(const uint8_t mpu6050Register, uint8_t buf[], uint8_t bufLength);

    bool setAccelScaleRange(const uint8_t accelScaleConfig);
};

} /* namespace mpu6050 */