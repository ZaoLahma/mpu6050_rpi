#include "mpu6050.h"

#include <iostream>
#include <thread>
#include <chrono>

#define UNUSED(arg) (void) arg;

#define CHECK(statement)if (!(statement)) { std::cout<<__FILE__<<", "<<__LINE__<<" "<<"CHECK - "<<#statement<<" FAILED "<<std::endl; return 1; }
#define CHECK_EQ(this, that)if (!(this == that)) { std::cout<<__FILE__<<", "<<__LINE__<<" "<<"CHECK_EQ - "<<#this<<" "<<"("<<this<<")"<<" != "<<#that<<" ("<<that<<")"<<std::endl; return 1; }
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

    std::cout<<"ENABLE TEMPERATURE"<<std::endl;
    CHECK(true == mpu.enableTemperatureSensor());

    DELAY(500ms);
    expectedRegValue = 0x0u;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    float temperature {0.0f};
    CHECK(true == mpu.getTemperature(temperature));
    std::cout<<"Temperature "<<temperature<<std::endl;

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

    std::cout<<"SLEEP"<<std::endl;
    CHECK(true == mpu.sleep());

    DELAY(500ms);
    expectedRegValue = 0x40;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    std::cout<<"Test application done\n";
    return 0;
}