#pragma once

#include <cstdint>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace mpu6050
{

/* Default device I2C address */
constexpr uint8_t MPU6050_DEFAULT_I2C_ADDR             {0x68};

/* Register bit masks */
constexpr uint8_t MPU6050_REG_DATA_RDY_EN_MASK         {1u << 0u};

constexpr uint8_t MPU6050_FIFO_OFLOW_INT_MASK          {1u << 4u};
constexpr uint8_t MPU6050_REG_DATA_RDY_INT_MASK        {1u << 0u};

constexpr uint8_t MPU6050_REG_FIFO_ENABLE_MASK         {1u << 6u};
constexpr uint8_t MPU6050_REG_FIFO_RESET_MASK          {1u << 2u};

constexpr uint8_t MPU6050_REG_SLEEP_MASK               {1u << 6u};
constexpr uint8_t MPU6050_REG_RESET_MASK               {1u << 7u};
constexpr uint8_t MPU6050_REG_TEMP_DISABLE_MASK        {1u << 4u};

constexpr uint8_t MPU6050_REG_FIFO_TEMP_ENABLE_MASK    {1u << 7u};
constexpr uint8_t MPU6050_REG_FIFO_ACCEL_ENABLE_MASK   {1u << 3u};

/* Registers */
constexpr uint8_t MPU6050_REG_USER_CONTROL             {0x6Au};

constexpr uint8_t MPU6050_REG_INT_ENABLE               {0x38u};
constexpr uint8_t MPU6050_REG_INT_STATUS               {0x3Au};

constexpr uint8_t MPU6050_REG_FIFO_ENABLE              {0x23u};
constexpr uint8_t MPU6050_REG_FIFO_COUNT_HIGH          {0x72u};
constexpr uint8_t MPU6050_REG_FIFO_COUNT_LOW           {0x73u};
constexpr uint8_t MPU6050_REG_FIFO_READ_WRITE          {0x74u};

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

constexpr uint8_t MPU6050_REG_GYRO_CONFIG              {0x1Bu};
constexpr uint8_t MPU6050_REG_GYRO_X_HIGH              {0x43u};
constexpr uint8_t MPU6050_REG_GYRO_X_LOW               {0x44u};
constexpr uint8_t MPU6050_REG_GYRO_Y_HIGH              {0x45u};
constexpr uint8_t MPU6050_REG_GYRO_Y_LOW               {0x46u};
constexpr uint8_t MPU6050_REG_GYRO_Z_HIGH              {0x47u};
constexpr uint8_t MPU6050_REG_GYRO_Z_LOW               {0x48u};

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

constexpr float MPU6050_GYRO_SCALE_250_FACTOR          {1.0f / 131.0f};
constexpr float MPU6050_GYRO_SCALE_500_FACTOR          {1.0f / 65.5f};
constexpr float MPU6050_GYRO_SCALE_1000_FACTOR         {1.0f / 32.8f};
constexpr float MPU6050_GYRO_SCALE_2000_FACTOR         {1.0f / 16.4f};

/* Sensor configuration */
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_2G  {0u << 3u};
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_4G  {1u << 3u};
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_8G  {2u << 3u};
constexpr uint8_t MPU6050_ACCEL_CONFIG_SCALE_RANGE_16G {3u << 3u};

constexpr uint8_t MPU6050_GYRO_CONFIG_SCALE_RANGE_250  {0u << 3u};
constexpr uint8_t MPU6050_GYRO_CONFIG_SCALE_RANGE_500  {1u << 3u};
constexpr uint8_t MPU6050_GYRO_CONFIG_SCALE_RANGE_1000 {2u << 3u};
constexpr uint8_t MPU6050_GYRO_CONFIG_SCALE_RANGE_2000 {3u << 3u};

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

    /* Sensor configuration */
    bool setAccelScaleRange2G();
    bool setAccelScaleRange4G();
    bool setAccelScaleRange8G();
    bool setAccelScaleRange16G();

    bool setGyroScaleRange250();
    bool setGyroScaleRange500();
    bool setGyroScaleRange1000();
    bool setGyroScaleRange2000();

    /* 
        Control the FIFO buffer for all (MPU6050 internal) sensors. Not really working yet.

        The intention is to spawn a thread that continuously reads the FIFO buffer on the MPU 6050
        and copies it into this device's memory. Then reads of sensor data through the API is directed
        into the buffer instead of to the MPU 6050.
    */
    bool enableFIFO();
    bool disableFIFO();
    bool getFIFOCount(uint16_t& count);

    /* Sensor data */
    bool getTemperature(float& temperature);
    bool getAcceleration(float& x, float& y, float& z);
    bool getGyro(float& x, float& y, float& z);

    /* Continuously read all raw data sensor values */
    bool enableSensorDataThread();
    bool disableSensorDataThread();

    /* General register write */
    bool writeRegister(const uint8_t mpu6050Register, const uint8_t value);

    /* General register read */
    bool readRegister(const uint8_t mpu6050Register, uint8_t& value);

    protected:
    bool setRegisterValue(const uint8_t mpu6050Register, const uint8_t bitmask, const bool set);

    bool getRegisterWord(const uint8_t mpu6050RegisterHigh, const uint8_t mpu6050RegisterLow, uint16_t& value);

    /* Read 2 bytes of data from two separate registers at addresses mpu6050Register and mpu6050Register + 1u, "probably" atomically */
    bool getRegisterWordBurstRead(const uint8_t mpu6050Register, uint16_t& value);

    int getRegisterData(const uint8_t mpu6050Register, uint8_t buf[], uint8_t bufLength);

    uint16_t toUint16(const uint8_t buf[], const uint8_t startPos);

    private:
    uint8_t m_i2cAddr;
    int m_i2cFileDescriptor;

    /* Raw sensor data buffer */
    static const uint8_t RAW_SENSOR_DATA_BUF_SIZE {14u};
    uint8_t RAW_SENSOR_DATA_BUF[RAW_SENSOR_DATA_BUF_SIZE];
    bool m_sensorReaderThreadEnabled {false};
    std::thread m_rawDataThread;
    std::mutex m_rawDataThreadMutex;
    std::condition_variable m_rawDataControlNotification;

    void sensorReader();

    /* FIFO stuff */
    static const uint8_t FIFO_BUF_SIZE {8u};
    uint8_t FIFO_BUF[FIFO_BUF_SIZE];
    bool m_fifoEnabled {false};
    std::thread m_fifoThread;
    std::mutex m_fifoThreadMutex;
    std::condition_variable m_fifoControlNotification;

    void fifoReader();

    /* Config stuff */
    bool setAccelScaleRange(const uint8_t accelScaleConfig);
    bool setGyroScaleRange(const uint8_t gyroScaleConfig);
};

} /* namespace mpu6050 */