#pragma once

#define DEFAULT            &to  LAYER_DEFAULT        // default
#define SNIPER             &to  LAYER_SNIPE          // sniper
#define DRAG_SCROLL        &mo  LAYER_SCROLL
#define DEVICE             &mo  LAYER_DEVICE
#define USER_MB5           &ltm LAYER_USER MB5       // config

#define _____xx_____       &none
#define ____________       &trans

#define ENCODER_UP         &kp C_VOL_UP &kp LC(TAB)
#define ENCODER_DOWN       &kp C_VOL_DN &kp LC(LS(TAB))
#define ENCODER_NONE       _____xx_____ _____xx_____

#define CYCLE_POLLRT       &rrl 1
#define TWIST_TOGGLE       &p2sm_twist_toggle
#define TWIST_UP           &scrlsens P2SM_INC 1
#define TWIST_DOWN         &scrlsens P2SM_DEC 1
#define POINT_UP           &sens     P2SM_INC 1
#define POINT_DOWN         &sens     P2SM_DEC 1

#define OUTPUT_TOG         &out OUT_TOG
#define BT_CLEAR           &bt  BT_CLR
#define BT_1               &bt  BT_SEL 0
#define BT_2               &bt  BT_SEL 1
#define BT_3               &bt  BT_SEL 2
#define BT_4               &bt  BT_SEL 3
#define BT_5               &bt  BT_SEL 4

#define SOFT_OFF           &soft_off
#define AF_TOGGLE          &af_toggle AF_TOG
#define ZMK_STUDIO         &studio_unlock
#define OS_SWAP            &bst_tog ZBS_TOG
#define MUI_BT             &zbs_adv

#define COLOR_OFF          00 00 00
#define COLOR_DEVICE       80 00 FF
#define COLOR_SNIPE        FF 00 00
#define COLOR_USER         00 80 FF
#define COLOR_BT_CHANGE    00 FF 40
#define PULSE_BT           50 175
#define PULSE_LAYER        60

#define TWIST_MPLY         1
#define TWIST_DIV          10

#define SCROLL_MPLY        1
#define SCROLL_DIV         3

#define SNIPE_MPLY         1
#define SNIPE_DIV          4