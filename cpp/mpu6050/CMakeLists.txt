add_library(mpu6050 "")

target_sources(mpu6050
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/inc/mpu6050.h
  PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src/mpu6050.cpp
  )
target_include_directories(mpu6050
  PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/inc
)
target_link_libraries(mpu6050
    i2c
)

set_target_properties(mpu6050 PROPERTIES LINKER_LANGUAGE CXX)