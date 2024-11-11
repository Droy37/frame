//
// Created by CLC on 2024/11/11.
//

#include "motor_pid.h"



PID::PID(float kp, float ki, float kd, float i_max, float out_max) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    i_max_ = i_max;
    out_max_ = out_max;
};

float PID::calc(float ref, float fdb) {
    last_err_ = err_;
    err_ = ref - fdb;
    err_sum_ += err_;
    if (err_sum_ > i_max_) {
        err_sum_ = i_max_;
    } else if (err_sum_ < -i_max_) {
        err_sum_ = -i_max_;
    }

    pout_ = kp_ * err_;
    iout_ = ki_ * err_sum_;
    dout_ = kd_ * (last_err_ - err_);

    output_ = pout_ + iout_+ dout_ ;
    if (output_ > out_max_) {
        output_ = out_max_;
    } else if (output_ < -out_max_) {
        output_ = -out_max_;
    }

    return output_;
}


PID pid_spd(10, 0, 0, 3000, 5000);
PID pid_pos(10, 0, 0, 3000, 5000);