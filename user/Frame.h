/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2024-12-09     杨金鹏       the first version
 */
#ifndef FRAME_H_
#define FRAME_H_
#include "GaitController.h"
typedef void (*CallBackFunc)(void);
typedef void (*InitFunc)(void);
typedef void (*Action)(void);
typedef void (*Gait_Action)(const MotionParams* params);

void FrameEntry(void);
void CtrlEntry(CallBackFunc CtrlMode);
void BluetoochMode(void);
void TestMode(void);
void BalanceMode(void);
void PidTest(void);
void Testfunc(void);


#endif /* HEXAPOD_FRAME_FRAME_H_ */
