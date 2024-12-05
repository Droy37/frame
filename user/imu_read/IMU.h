//
// Created by CLC on 2024/11/13.
//

#ifndef IMU_H
#define IMU_H
#include <stdint.h>


typedef struct
{
    float roll, pitch, yaw;
    float r_acc, r_gyro;
    float p_acc, p_gyro;
    float y_gyro;
}IMU;

void BMI088_ACCEL_NS_L();
void BMI088_ACCEL_NS_H();
void BMI088_GYRO_NS_L();
void BMI088_GYRO_NS_H();
void BMI088_ReadReg_ACCEL(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_ReadReg_GYRO(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_WriteReg(uint8_t reg, uint8_t write_data);

void BMI088_Init();

void IMU_calc();
extern IMU imu;
#endif //IMU_H
