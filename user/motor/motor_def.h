//
// Created by CLC on 2024/11/11.
//


#ifndef MOTOR_DEF_H
#define MOTOR_DEF_H
#include <stdint.h>


class Motor {
public:
    float ratio_; // 电机减速比

    float angle_; // deg 输出端累计转动角度
    float delta_angle_; // deg 输出端新转动的角度

    float ecd_angle_; // deg 当前电机编码器角度
    float last_ecd_angle_; // deg 上次电机编码器角度
    float delta_ecd_angle_; // deg 编码器端新转动的角度

    float rotate_speed_; // dps 反馈转子转速

    float current_; // A 反馈转矩电流
    float temp_; // °C 反馈电机温度

    float max_;
    float min_;

    struct {
        float fdb_;
        float target_;
        float ff_;
        float output_;
    } control_data;

    void canRxMsgCallback(uint8_t rx_data[8]);

};

class M3508 : public Motor
{
public:
    M3508() : Motor()
    {
        ratio_ = 3591.0 / 187;
        angle_ = 0;
        delta_angle_ = 0;
        ecd_angle_ = 0;
        last_ecd_angle_ = 0;
        delta_ecd_angle_ = 0;
        rotate_speed_ = 0;
        current_ = 0;
        temp_ = 25;
        max_ = 100;
        min_ = 0;
    }
};

class M2006 : public Motor
{
public:
    M2006() : Motor()
    {
        ratio_ = 36;
        angle_ = 0;
        delta_angle_ = 0;
        ecd_angle_ = 0;
        last_ecd_angle_ = 0;
        delta_ecd_angle_ = 0;
        rotate_speed_ = 0;
        current_ = 0;
        temp_ = 25;
        max_ = 100;
        min_ = 0;
    }
    uint16_t updateMotor(float rc_input, Motor motor);
};

class M6020 : public Motor
{
public:
    M6020() : Motor()
    {
        ratio_ = 1;
        angle_ = 0;
        delta_angle_ = 0;
        ecd_angle_ = 0;
        last_ecd_angle_ = 0;
        delta_ecd_angle_ = 0;
        rotate_speed_ = 0;
        current_ = 0;
        temp_ = 25;
        max_ = 185;
        min_ = 145;
    }
};

void updateMotorPitch(float target);
void updateMotorYaw(float target);

extern M6020 motor_pitch;
extern M6020 motor_yaw;
extern uint8_t rx_data[8];
extern uint8_t tx_data[8];
#endif //MOTOR_DEF_H