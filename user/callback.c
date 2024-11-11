//
// Created by CLC on 2024/11/11.
//

#include "usart.h"
#include "remotecontrol/rc_def.h"
#include "remotecontrol/rc_func.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart3) {
        HAL_UARTEx_ReceiveToIdle_DMA(&huart3, buffer, sizeof(buffer));
        for(int i = 0; i < 18; i++) {
            rc_data[i] = buffer[i];
        }
        dataProcess(rc_data);
    }
}
