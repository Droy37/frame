//
// Created by CLC on 2024/11/11.
//

#ifndef RC_DEF_H
#define RC_DEF_H

#include <stdint.h>

typedef enum {
    up = 1,
    down,
    mid,
  } RCSwitchState_e;

typedef enum {
    release,
    press,
  } MouseState_e;

typedef struct{

    struct{
        float r_row;
        float r_col;
        float l_row;
        float l_col;
    }channel_;
    struct
    {
        RCSwitchState_e s1;
        RCSwitchState_e s2;
    }switch_;
    struct{
        float x;
        float y;
        float z;
        MouseState_e press_l;
        MouseState_e press_r;
    }mouse_;
    struct{
        uint16_t v;
    }key_;

}RC_Ctl_t;

extern RC_Ctl_t RC_CtrlData;
extern uint8_t buffer[18];
extern uint8_t rc_data[18];

#endif //RC_DEF_H
