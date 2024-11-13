//
// Created by CLC on 2024/11/11.
//

#include "motor_def.h"
#include "motor_pid.h"
#include "../uni_func.h"
#include "../Inc/main.h"

void Motor::canRxMsgCallback(uint8_t* rx_data){
    uint16_t ecd_angle_mid = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle_mid, 0, 8191, 0.0, 360.0);

    rotate_speed_ = float((rx_data[2] << 8) | rx_data[3]);

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

void updateMotorPitch(float target) {
    //motor_pitch.control_data.target_ = motor_pitch.control_data.fdb_ + rc_input * 3.0f;
    motor_pitch.control_data.target_ = target;

    if (motor_pitch.control_data.target_ > motor_pitch.max_) motor_pitch.control_data.target_ = motor_pitch.max_;
    if (motor_pitch.control_data.target_ < motor_pitch.min_) motor_pitch.control_data.target_ = motor_pitch.min_;

    float err_ = motor_pitch.control_data.target_ - motor_pitch.control_data.fdb_;
    if (err_ > 180) motor_pitch.control_data.fdb_ += 360;
    else if (err_ < -180) motor_pitch.control_data.fdb_ -= 360;

    float target_speed = pid_pos_pitch.calc(motor_pitch.control_data.target_, motor_pitch.control_data.fdb_);

    motor_pitch.control_data.output_ = pid_spd_pitch.calc(target_speed, motor_pitch.rotate_speed_) + motor_pitch.control_data.ff_;
}

// uint16_t updateMotorYaw(float target) {
//     //pid_pos_yaw.ref_ = pid_pos_yaw.fdb_ + rc_input * 3.0f;
//
//     if (pid_pos_yaw.ref_ > motor_yaw.max_) pid_pos_yaw.ref_ = motor_yaw.max_;
//     if (pid_pos_yaw.ref_ < motor_yaw.min_) pid_pos_yaw.ref_ = motor_yaw.min_;
//
//     float err_ = pid_pos_yaw.ref_ - pid_pos_yaw.fdb_;
//     if (err_ > 180) pid_pos_yaw.fdb_ += 360;
//     else if (err_ < -180) pid_pos_yaw.fdb_ -= 360;
//
//     float target_speed = pid_pos_yaw.calc(pid_pos_yaw.ref_, pid_pos_yaw.fdb_);
//
//     pid_spd_yaw.ref_ = target_speed;
//     pid_spd_yaw.fdb_ = motor_yaw.rotate_speed_;
//
//     return uint16_t(pid_spd_yaw.calc(pid_spd_yaw.ref_, pid_spd_yaw.fdb_));
// }


M2006 motor_pitch;
M3508 motor_yaw;
uint8_t rx_data[8];
uint8_t tx_data[8];
