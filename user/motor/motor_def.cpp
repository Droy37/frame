//
// Created by CLC on 2024/11/11.
//

#include "motor_def.h"
#include "../uni_func.h"

void Motor::canRxMsgCallback(uint8_t* rx_data){
    uint16_t ecd_angle_mid = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle_mid, 0, 8191, 0.0, 360.0);

    int16_t rotate_speed_mid = (rx_data[2] << 8) | rx_data[3];
    rotate_speed_ = float(rotate_speed_mid);

    int16_t current_mid = (rx_data[4] << 8) | rx_data[5];
    current_ = linearMapping(current_mid, -16384, 16384, -20.0, 20.0);

    uint8_t temp_mid = rx_data[6];
    temp_ = float(temp_mid);


    delta_ecd_angle_ = ecd_angle_ - last_ecd_angle_;
    if (delta_ecd_angle_ < 0) {
        delta_ecd_angle_ += 360;
    }
    last_ecd_angle_ = ecd_angle_;


    delta_angle_ = delta_ecd_angle_;
    angle_ += delta_angle_;
}


M2006 motor_pitch;
M3508 motor_yaw;
uint8_t rx_data[8];
uint8_t tx_data[8];
