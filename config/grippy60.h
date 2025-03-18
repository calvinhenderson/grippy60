#pragma once

// LAYERS
#define MACOS 0
#define LINUX 1
#define BASE_LAYERS MACOS LINUX

// TIMEOUTS
#define TIMEOUT_SMALL   80
#define TIMEOUT_NORMAL 150
#define TIMEOUT_LONG   250
#define TIMEOUT_XLONG 1500

// Labels for switch directions according to the matrix

/* FINGER ORIENTATION         x x  */
/*            ^ [R]ight         x  */
/*            |                 x  */
/*  [B]ack    |   [F]orward     x  */
/*       <----+----->              */
/*            | \                  */
/*     [L]eft |  \ [D]own          */
/*            v   v                */

// Outer thumb
#define L_OT_F 0
#define R_OT_F 1
#define L_OT_L 2
#define L_OT_D 3
#define L_OT_R 4
#define R_OT_L 5
#define R_OT_D 6
#define R_OT_R 7
#define L_OT_B 8
#define R_OT_B 9

// Inner thumb
#define L_IT_F 10
#define R_IT_F 11
#define L_IT_L 12
#define L_IT_D 13
#define L_IT_R 14
#define R_IT_L 15
#define R_IT_D 16
#define R_IT_R 17
#define L_IT_B 18
#define R_IT_B 19

// Index finger
#define L_IF_F 20
#define R_IF_F 21
#define L_IF_L 22
#define L_IF_D 23
#define L_IF_R 24
#define R_IF_L 25
#define R_IF_D 26
#define R_IF_R 27
#define L_IF_B 28
#define R_IF_B 29

// Middle finger
#define L_MF_F 30
#define R_MF_F 31
#define L_MF_L 32
#define L_MF_D 33
#define L_MF_R 34
#define R_MF_L 35
#define R_MF_D 36
#define R_MF_R 37
#define L_MF_B 38
#define R_MF_B 39

// Ring finger
#define L_RF_F 40
#define R_RF_F 41
#define L_RF_L 42
#define L_RF_D 43
#define L_RF_R 44
#define R_RF_L 45
#define R_RF_D 46
#define R_RF_R 47
#define L_RF_B 48
#define R_RF_B 49

// Pinky finger
#define L_PF_F 50
#define R_PF_F 51
#define L_PF_L 52
#define L_PF_D 53
#define L_PF_R 54
#define R_PF_L 55
#define R_PF_D 56
#define R_PF_R 57
#define L_PF_B 58
#define R_PF_B 59
