//
// Created by CLC on 2024/11/11.
//

#ifndef MOTOR_PID_H
#define MOTOR_PID_H


class PID {
public:
    PID(float kp, float ki, float kd, float i_max, float out_max);
    float calc(float ref, float fdb);

    float kp_,ki_, kd_;
    float i_max_,out_max_;
    float output_;
    float ref_, fdb_;
    float err_,err_sum_, last_err_;
    float pout_, iout_, dout_;
};
extern PID pid_spd, pid_pos;


#endif //MOTOR_PID_H
