//
// Created by CLC on 2024/11/11.
//

#include "can.h"
#include "tim.h"
#include "usart.h"
#include "remotecontrol/rc_def.h"
#include "motor/motor_def.h"
#include "uni_func.h"

CAN_FilterTypeDef FilterConfig = {0, 0, 0, 0, CAN_FILTER_FIFO0, 14, CAN_FILTERMODE_IDMASK, CAN_FILTERSCALE_32BIT, ENABLE};
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