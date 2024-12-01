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
float target_pitch = 230, target_yaw = 70;
float fdb_pitch, fdb_yaw;
void MainLoop(){
//    HAL_IWDG_Refresh(&hiwdg);
    dataProcess(rc_data);
    BMI088_ReadReg_ACCEL(0x12, &return_data, 6);
    BMI088_ReadReg_GYRO(0x00, &return_data, 8);

    if (RC_CtrlData.switch_.s2 != down){
        motor_pitch.control_data.fdb_ = motor_pitch.ecd_angle_;
        fdb_pitch = motor_pitch.ecd_angle_;
        updateMotorPitch(RC_CtrlData.channel_.l_col);
        //updateMotorPitch(target_pitch);
        output = uint16_t(motor_pitch.control_data.output_);
        tx_data[0] = uint8_t(output >> 8);
        tx_data[1] = uint8_t(output & 0xFF);

        motor_yaw.control_data.fdb_ = motor_yaw.ecd_angle_;
        fdb_yaw = motor_yaw.ecd_angle_;
        updateMotorYaw(RC_CtrlData.channel_.l_row);
        //updateMotorYaw(target_yaw);
        output = uint16_t(motor_yaw.control_data.output_);
        tx_data[4] = uint8_t(output >> 8);
        tx_data[5] = uint8_t(output & 0xFF);

        while(HAL_CAN_IsTxMessagePending(&hcan1, TxMailbox)){}
        if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, tx_data, &TxMailbox)!=HAL_OK){}

    }
}


#ifdef __cplusplus
}
#endif

