//
// Created by CLC on 2024/11/11.
//

#include "uni_func.h"
#include "can.h"
#include "motor/motor_def.h"
#include "motor/motor_pid.h"
#include "remotecontrol/rc_def.h"
#include "remotecontrol/rc_func.h"

CAN_TxHeaderTypeDef TxHeader ={0x1FF,0,CAN_ID_STD,CAN_RTR_DATA, 8, DISABLE};
uint32_t TxMailbox = CAN_TX_MAILBOX0;


float linearMapping(float in, float in_min, float in_max, float out_min, float out_max){
    return (in - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

uint16_t Calculate(float ref, Motor motor){
    pid_pos.ref_ = ref;
    pid_pos.fdb_ = motor.ecd_angle_;


    float err_ = pid_pos.ref_ - pid_pos.fdb_;
    if (err_ > 180) {
        pid_pos.fdb_ += 360;
    } else if (err_ < -180) {
        pid_pos.fdb_ -= 360;
    }
    float target_speed = pid_pos.calc(pid_pos.ref_, pid_pos.fdb_);

    pid_spd.ref_ = target_speed;
    pid_spd.fdb_ = motor.rotate_speed_;

    return uint16_t(pid_spd.calc(pid_spd.ref_, pid_spd.fdb_));
}


void MainLoop(){
    dataProcess(rc_data);


    uint16_t output = Calculate(RC_CtrlData.channel_.l_col, motor_pitch);
    tx_data[0] = uint8_t(output >> 8);   //待改
    tx_data[1] = uint8_t(output & 0xFF);
    TxHeader.StdId = 0x1FF;
    TxHeader.ExtId = 0;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);

    output = Calculate(RC_CtrlData.channel_.l_row, motor_yaw);
    tx_data[0] = uint8_t(output >> 8);   //待改
    tx_data[1] = uint8_t(output & 0xFF);
    TxHeader.StdId = 0x1FF;
    TxHeader.ExtId = 0;
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);

}
