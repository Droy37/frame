//
// Created by CLC on 2024/11/11.
//

#include "rc_func.h"
#include "stdint.h"
#include "rc_def.h"
#include "../uni_func.h"
#include "../motor/motor_def.h"

void dataProcess(const uint8_t pData[18]) {

    uint16_t r_row = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
    uint16_t r_col = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    uint16_t l_row = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((int16_t)pData[4] << 10)) & 0x07FF;
    uint16_t l_col = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
    RC_CtrlData.channel_.r_row = linearMapping(r_row, 364, 1684, -1, 1);
    RC_CtrlData.channel_.r_col = linearMapping(r_col, 364, 1684, -1, 1);
    RC_CtrlData.channel_.l_row = linearMapping(l_row, 364, 1684, -1, 1);  //待改
    RC_CtrlData.channel_.l_col = linearMapping(l_col, 364, 1684, motor_pitch.min, motor_pitch.max);

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

