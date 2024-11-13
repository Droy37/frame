//
// Created by CLC on 2024/11/11.
//

#include "uni_func.h"
#include "can.h"
#include "iwdg.h"
#include "motor/motor_def.h"
#include "remotecontrol/rc_def.h"

CAN_TxHeaderTypeDef TxHeader ={0x1FF,0,CAN_ID_STD,CAN_RTR_DATA, 8, DISABLE};
uint32_t TxMailbox = CAN_TX_MAILBOX0;


float linearMapping(float in, float in_min, float in_max, float out_min, float out_max){
    return (in - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
}

void dataProcess(const uint8_t pData[18]) {

    uint16_t r_row = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    uint16_t r_col = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    uint16_t l_row = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
    uint16_t l_col = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    RC_CtrlData.channel_.r_row = linearMapping(r_row, 364, 1684, -1, 1);
    RC_CtrlData.channel_.r_col = linearMapping(r_col, 364, 1684, -1, 1);
    RC_CtrlData.channel_.l_row = linearMapping(l_row, 364, 1684, -1, 1);
    RC_CtrlData.channel_.l_col = linearMapping(l_col, 364, 1684, -1, 1);

    RC_CtrlData.switch_.s1 = (RCSwitchState_e)(((pData[5] >> 4) & 0x000C) >> 2);
    RC_CtrlData.switch_.s2 = (RCSwitchState_e)((pData[5] >> 4) & 0x0003);

    int16_t mouse_x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
    int16_t mouse_y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
    int16_t mouse_z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
    RC_CtrlData.mouse_.x = linearMapping(mouse_x, -32768, 32768, -1, 1);
    RC_CtrlData.mouse_.y = linearMapping(mouse_y, -32768, 32768, -1, 1);
    RC_CtrlData.mouse_.z = linearMapping(mouse_z, -32768, 32768, -1, 1);

    RC_CtrlData.mouse_.press_l = (MouseState_e)pData[12];
    RC_CtrlData.mouse_.press_r = (MouseState_e)pData[13];
    RC_CtrlData.key_.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
}


uint16_t output;

void MainLoop(){
    HAL_IWDG_Refresh(&hiwdg);
    dataProcess(rc_data);

    if (RC_CtrlData.switch_.s2 != down){
        motor_pitch.control_data.fdb_ = motor_pitch.ecd_angle_;
        updateMotorPitch(RC_CtrlData.channel_.l_col);
        tx_data[0] = uint8_t(motor_pitch.control_data.output_ >> 8);
        tx_data[1] = uint8_t(motor_pitch.control_data.output_ & 0xFF);
        TxHeader.StdId = 0x1FF;
        TxHeader.ExtId = 0;
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);

        output = updateMotorYaw(RC_CtrlData.channel_.l_row);
        tx_data[0] = uint8_t(output >> 8);
        tx_data[1] = uint8_t(output & 0xFF);
        TxHeader.StdId = 0x1FF;
        TxHeader.ExtId = 0;
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);
    }
}
