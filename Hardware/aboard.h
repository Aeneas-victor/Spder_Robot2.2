/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2025-02-18     杨金鹏       the first version
 */
#ifndef _ABOARD_H_
#define _ABOARD_H_
#include "Action.h"
#include "stm32f4xx.h"
#define KEY_PORT GPIOE
#define KEY0_PIN GPIO_Pin_4
#define KEY1_PIN GPIO_Pin_3

void Board_Init(void);

#endif /* HEXAPOD_HARDWARE_ABOARD_H_ */
