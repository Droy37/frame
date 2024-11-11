//
// Created by CLC on 2024/11/11.
//

#ifndef REMOTECONTROL_H
#define REMOTECONTROL_H
#include "stdint.h"

float linearMapping(float in, float in_min, float in_max, float out_min, float out_max);
void dataProcess(const uint8_t pData[18]);

#endif //REMOTECONTROL_H
