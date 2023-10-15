#include "mpu6050.h"

#include <iostream>
#include <thread>
#include <chrono>

#define UNUSED(arg) (void) arg;

#define CHECK(statement)if (!(statement)) { std::cout<<__FILE__<<", "<<__LINE__<<" "<<"CHECK - "<<#statement<<" FAILED"<<std::endl; return 1; }
#define CHECK_EQ(this, that)if (!(this == that)) { std::cout<<__FILE__<<", "<<__LINE__<<" "<<"CHECK_EQ - "<<#this<<" "<<"("<<this<<")"<<" != "<<#that<<" ("<<that<<") FAILED"<<std::endl; return 1; }
#define DELAY(time) std::this_thread::sleep_for(time)

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    UNUSED(argc);
    UNUSED(argv);
    std::cout<<"Test application starting\n";

    uint8_t expectedRegValue {0x0};
    uint8_t regValue {0};

    mpu6050::Mpu6050 mpu;
    mpu.resetRegisters();

    DELAY(500ms);
    expectedRegValue = 0x40; /* Init state */
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    std::cout<<"WAKEUP"<<std::endl;
    CHECK(true == mpu.wakeup());

    DELAY(500ms);
    expectedRegValue = 0x0;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    float accelX {0.0f};
    float accelY {0.0f};
    float accelZ {0.0f};
    CHECK(true == mpu.getAcceleration(accelX, accelY, accelZ));
    std::cout<<"X "<<accelX<<", Y "<<accelY<<", Z "<<accelZ<<std::endl;

    CHECK(true == mpu.setAccelScaleRange4G());
    DELAY(500ms);
    CHECK(true == mpu.getAcceleration(accelX, accelY, accelZ));
    std::cout<<"X "<<accelX<<", Y "<<accelY<<", Z "<<accelZ<<std::endl;

    CHECK(true == mpu.setAccelScaleRange8G());
    DELAY(500ms);
    CHECK(true == mpu.getAcceleration(accelX, accelY, accelZ));
    std::cout<<"X "<<accelX<<", Y "<<accelY<<", Z "<<accelZ<<std::endl;   

    CHECK(true == mpu.setAccelScaleRange16G());
    DELAY(500ms);
    CHECK(true == mpu.getAcceleration(accelX, accelY, accelZ));
    std::cout<<"X "<<accelX<<", Y "<<accelY<<", Z "<<accelZ<<std::endl;

    CHECK(true == mpu.setAccelScaleRange4G());
    DELAY(500ms);

    std::cout<<"ENABLE TEMPERATURE"<<std::endl;
    CHECK(true == mpu.enableTemperatureSensor());

    DELAY(500ms);
    expectedRegValue = 0x0u;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    float temperature {0.0f};
    CHECK(true == mpu.getTemperature(temperature));
    std::cout<<"Temperature "<<temperature<<std::endl;

    std::cout<<"GYRO"<<std::endl;

    CHECK(true == mpu.setGyroScaleRange250());
    DELAY(500ms);
    float gyroX {0};
    float gyroY {0};
    float gyroZ {0};
    CHECK(true == mpu.getGyro(gyroX, gyroY, gyroZ));
    std::cout<<"Gyro "<<gyroX<<", "<<gyroY<<", "<<gyroZ<<std::endl;

    CHECK(true == mpu.setGyroScaleRange500());
    DELAY(500ms);
    CHECK(true == mpu.getGyro(gyroX, gyroY, gyroZ));
    std::cout<<"Gyro "<<gyroX<<", "<<gyroY<<", "<<gyroZ<<std::endl;

    CHECK(true == mpu.setGyroScaleRange1000());
    DELAY(500ms);
    CHECK(true == mpu.getGyro(gyroX, gyroY, gyroZ));
    std::cout<<"Gyro "<<gyroX<<", "<<gyroY<<", "<<gyroZ<<std::endl;

    CHECK(true == mpu.setGyroScaleRange2000());
    DELAY(500ms);
    CHECK(true == mpu.getGyro(gyroX, gyroY, gyroZ));
    std::cout<<"Gyro "<<gyroX<<", "<<gyroY<<", "<<gyroZ<<std::endl;

    DELAY(500ms);
    uint16_t fifoCount {0u};
    UNUSED(fifoCount);
    CHECK(true == mpu.enableFIFO());
    
    for (uint16_t i {0u}; i < 100u; ++i)
    {
        CHECK(true == mpu.getTemperature(temperature));
        std::cout<<"Temperature from FIFO: "<<temperature<<std::endl;
        DELAY(10ms);
    }

    DELAY(500ms);
    CHECK(true == mpu.disableFIFO());
    DELAY(500ms);

    CHECK(true == mpu.enableSensorDataThread());

    for (uint16_t i {0u}; i < 100u; ++i)
    {
        CHECK(true == mpu.getTemperature(temperature));
        std::cout<<"Temperature from sensor thread: "<<temperature<<std::endl;
        DELAY(10ms);
    }
    DELAY(500ms);

    mpu.disableSensorDataThread();

    DELAY(500ms);
    
    std::cout<<"SLEEP"<<std::endl;
    CHECK(true == mpu.sleep());

    DELAY(500ms);
    expectedRegValue = 0x40;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    std::cout<<"Test application done result SUCCESS\n";
    return 0;
}