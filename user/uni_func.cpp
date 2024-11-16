//
// Created by CLC on 2024/11/11.
//

#ifdef __cplusplus
extern "C" {
#endif


#include "uni_func.h"
#include "can.h"
#include "iwdg.h"
#include "imu_read/IMU.h"
#include "motor/motor_def.h"
#include "remotecontrol/rc_def.h"
#include "main.h"
#include "remotecontrol/rc_func.h"

CAN_TxHeaderTypeDef TxHeader ={0x1FF,0,CAN_ID_STD,CAN_RTR_DATA, 8, DISABLE};
uint32_t TxMailbox = CAN_TX_MAILBOX0;

uint16_t output;
extern uint8_t return_data;
float target = 168;
float feedback;
void MainLoop(){
//    HAL_IWDG_Refresh(&hiwdg);
    dataProcess(rc_data);
    BMI088_ReadReg_ACCEL(0x12, &return_data, 6);
    BMI088_ReadReg_GYRO(0x00, &return_data, 8);

    //if (RC_CtrlData.switch_.s2 != down){
    motor_pitch.control_data.fdb_ = motor_pitch.ecd_angle_;
    feedback = motor_pitch.ecd_angle_;
    updateMotorPitch(target);
    output = uint16_t(motor_pitch.control_data.output_);

    tx_data[0] = uint8_t(output >> 8);
    tx_data[1] = uint8_t(output & 0xFF);
    tx_data[2] = uint8_t(output >> 8);
    tx_data[3] = uint8_t(output & 0xFF);
    tx_data[4] = uint8_t(output >> 8);
    tx_data[5] = uint8_t(output & 0xFF);
    tx_data[6] = uint8_t(output >> 8);
    tx_data[7] = uint8_t(output & 0xFF);
    TxHeader.StdId = 0x1FF;

    while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox))
    {
        HAL_CAN_StateTypeDef b=HAL_CAN_GetState(&hcan1);
    }
    if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox)!=HAL_OK){};
    // TxHeader.StdId = 0x2FF;
    // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);

        // output = updateMotorYaw(RC_CtrlData.channel_.l_row);
        // tx_data[0] = uint8_t(output >> 8);
        // tx_data[1] = uint8_t(output & 0xFF);
        // TxHeader.StdId = 0x1FF;
        // TxHeader.ExtId = 0;
        // HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox);
    //}
}


#ifdef __cplusplus
}
#endif

