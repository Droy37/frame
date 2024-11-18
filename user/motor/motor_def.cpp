//
// Created by CLC on 2024/11/11.
//

#ifdef __cplusplus
extern "C" {
#endif


#include "motor_def.h"
#include "motor_pid.h"
#include "../uni_func.h"
#include "math.h"
#include "../imu_read/IMU.h"
void Motor::canRxMsgCallback(uint8_t* rx_data){
    uint16_t ecd_angle_mid = (rx_data[0] << 8) | rx_data[1];
    ecd_angle_ = linearMapping(ecd_angle_mid, 0, 8191, 0.0, 360.0);

    int16_t rotate_speed_mid = int16_t((rx_data[2] << 8) | rx_data[3]);
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

extern float accel[3];
extern float gyro[3];
extern float pi;
uint16_t k_ff = 0x270;

void updateMotorPitch(float input) {
    motor_pitch.control_data.target_ = motor_pitch.control_data.fdb_ - input * 3.0f;
    //motor_pitch.control_data.target_ = input;
    motor_pitch.control_data.fdb_ = motor_pitch.ecd_angle_;

    if (motor_pitch.control_data.target_ > motor_pitch.max_) motor_pitch.control_data.target_ = motor_pitch.max_;
    if (motor_pitch.control_data.target_ < motor_pitch.min_) motor_pitch.control_data.target_ = motor_pitch.min_;

    float err_ = motor_pitch.control_data.target_ - motor_pitch.control_data.fdb_;
    if (err_ > 180) motor_pitch.control_data.fdb_ += 360;
    else if (err_ < -180) motor_pitch.control_data.fdb_ -= 360;

    float target_speed = pid_pos_pitch.calc(motor_pitch.control_data.target_, motor_pitch.control_data.fdb_);

    //calculateAngle();
    motor_pitch.control_data.ff_ = k_ff * cos((motor_pitch.ecd_angle_-168.5) * pi / 180);
    motor_pitch.control_data.output_ = pid_spd_pitch.calc(target_speed, motor_pitch.rotate_speed_) + motor_pitch.control_data.ff_;
}



M6020 motor_pitch;
M6020 motor_yaw;
uint8_t rx_data[8];
uint8_t tx_data[8];


#ifdef __cplusplus
}
#endif
