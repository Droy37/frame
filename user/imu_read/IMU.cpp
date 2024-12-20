//
// Created by CLC on 2024/11/13.
//


#ifdef __cplusplus
extern "C" {
#endif

#include "IMU.h"
#include "math.h"
#include "spi.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#define M_PI 3.14159265358979323846

extern float accel[3];
extern float gyro[3];

extern uint8_t i;
extern uint8_t imu_rx_data[8];
extern uint8_t imu_tx_data;
extern uint8_t range;

void BMI088_ACCEL_NS_L() {
    HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H() {
    HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L() {
    HAL_GPIO_WritePin(CS_GYR_GPIO_Port, CS_GYR_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H() {
    HAL_GPIO_WritePin(CS_GYR_GPIO_Port, CS_GYR_Pin, GPIO_PIN_SET);
}

void BMI088_ReadReg_ACCEL(uint8_t reg, uint8_t *return_data, uint8_t length) {
    BMI088_ACCEL_NS_L();
    imu_tx_data = (reg | 0x80);
    HAL_SPI_Transmit(&hspi1, &imu_tx_data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);

    i = 0;
    while (i < length) {
        HAL_SPI_Receive(&hspi1, return_data, 1, 1000);
        while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
        imu_rx_data[i] = *return_data;
        i++;
    }
    BMI088_ACCEL_NS_H();
    accel[0] = (int16_t)((imu_rx_data[1] << 8) | imu_rx_data[0]) *10*pow(2,(range+1))*1.5/32768;
    accel[1] = (int16_t)((imu_rx_data[3] << 8) | imu_rx_data[2]) *10*pow(2,(range+1))*1.5/32768;
    accel[2] = (int16_t)((imu_rx_data[5] << 8) | imu_rx_data[4]) *10*pow(2,(range+1))*1.5/32768;
}

void BMI088_ReadReg_GYRO(uint8_t reg, uint8_t *return_data, uint8_t length) {
    BMI088_GYRO_NS_L();
    imu_tx_data = (reg | 0x80);
    HAL_SPI_Transmit(&hspi1, &imu_tx_data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);

    i = 0;
    while (i < length)
    {
        HAL_SPI_Receive(&hspi1, return_data, 1, 1000);
        while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_RX);
        imu_rx_data[i] = *return_data;
        i++;
    }
    BMI088_GYRO_NS_H();

    gyro[0] = (float)((int16_t)((imu_rx_data[3]) << 8) | imu_rx_data[2])*2000/32767 ;
    gyro[1] = (float)((int16_t)((imu_rx_data[5]) << 8) | imu_rx_data[4])*2000/32767 ;
    gyro[2] = (float)((int16_t)((imu_rx_data[7]) << 8) | imu_rx_data[6])*2000/32767 ;
}

void BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    BMI088_ACCEL_NS_L();
    imu_tx_data = (reg & 0x7F);
    HAL_SPI_Transmit(&hspi1, &imu_tx_data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);

    imu_tx_data = write_data;
    HAL_SPI_Transmit(&hspi1, &imu_tx_data, 1, 1000);
    while(HAL_SPI_GetState(&hspi1)==HAL_SPI_STATE_BUSY_TX);

    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}

void BMI088_Init() {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Soft Reset GYRO
    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    BMI088_GYRO_NS_H();

    // Switch ACCEL to Normal Mode
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_WriteReg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}

IMU imu;

float k = 0.0004;
float imu_dt = 0.001;
void IMU_calc()
{
    imu.r_acc = atan2(accel[1], accel[2]) * 180.0 / M_PI;
    imu.p_acc = -atan2(accel[0], sqrt(accel[1]*accel[1] + accel[2]*accel[2])) * 180.0 / M_PI;

    float roll = imu.roll * M_PI / 180;
    float pitch = imu.pitch * M_PI / 180;
    float yaw_gyro = sin(roll) / cos(pitch) * gyro[1] + cos(roll) / cos(pitch) * gyro[2];
    float roll_gyro = gyro[0] + sin(pitch) * sin(roll) / cos(pitch) * gyro[1] + cos(roll) * sin(pitch) / cos(pitch) * gyro[2];
    float pitch_gyro = cos(roll) * gyro[1] - sin(roll) * gyro[2];

    imu.yaw = imu.yaw + yaw_gyro * imu_dt;
    imu.roll = (imu.roll + roll_gyro * imu_dt) * (1 - k) + imu.r_acc * k;
    imu.pitch = (imu.pitch + pitch_gyro * imu_dt) * (1 - k) + imu.p_acc * k;
}


#ifdef __cplusplus
}
#endif
