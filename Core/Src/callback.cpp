//
// Created by CLC on 2024/11/15.
//
#ifdef __cplusplus
extern "C" {
#endif


#include "can.h"
#include "tim.h"
#include "usart.h"
#include "../../user/remotecontrol/rc_def.h"
#include "../../user/motor/motor_def.h"
#include "../../user/uni_func.h"

CAN_RxHeaderTypeDef RxHeader;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buffer, sizeof(buffer));
        for(int i = 0; i < 18; i++) {
            rc_data[i] = buffer[i];
        }
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, rx_data);
    motor_pitch.canRxMsgCallback(rx_data);
    motor_yaw.canRxMsgCallback(rx_data);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim1) {
        MainLoop();
    }
}

#ifdef __cplusplus
}
#endif

