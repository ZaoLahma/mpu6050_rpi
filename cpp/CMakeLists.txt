find_package(Threads)

add_executable(test_mpu6050
    test_application/main.cpp
)
target_compile_options(test_mpu6050 PRIVATE -Werror -Wall -Wextra -Wconversion)

add_subdirectory(mpu6050)

# Link the test executable
target_link_libraries(test_mpu6050
    PRIVATE 
        mpu6050
        Threads::Threads
)