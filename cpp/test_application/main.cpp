#include "mpu6050.h"

#include <iostream>
#include <thread>
#include <chrono>

#define UNUSED(arg) (void) arg;

#define CHECK(statement)if (!(statement)) { std::cout<<__FILE__<<", "<<__LINE__<<" "<<"CHECK - "<<#statement<<" FAILED "<<std::endl; return 1; }
#define CHECK_EQ(this, that)if (!(this == that)) { std::cout<<__FILE__<<", "<<__LINE__<<" "<<"CHECK_EQ - "<<#this<<" "<<"("<<this<<")"<<" != "<<#that<<" ("<<that<<")"<<std::endl; return 1; }

using namespace std::chrono_literals;
int main(int argc, char **argv)
{
    UNUSED(argc);
    UNUSED(argv);
    std::cout<<"Test application starting\n";

    int expectedRegValue = 0x0;
    int regValue = 0;

    mpu6050::Mpu6050 mpu;
    mpu.resetRegisters();

    std::this_thread::sleep_for(500ms);
    expectedRegValue = 0x40; /* Init state */
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    std::cout<<"WAKEUP"<<std::endl;
    CHECK(true == mpu.wakeup());

    std::this_thread::sleep_for(500ms);
    expectedRegValue = 0x0;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    std::cout<<"SLEEP"<<std::endl;
    CHECK(true == mpu.sleep());

    std::this_thread::sleep_for(500ms);
    expectedRegValue = 0x40;
    CHECK(true == mpu.readRegister(mpu6050::MPU6050_REG_PWR_PGMT, regValue));
    CHECK_EQ(expectedRegValue, regValue);

    std::cout<<"Test application done\n";
    return 0;
}