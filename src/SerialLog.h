/* Copyright (C) 2020 Mads Bornebusch and Kristian Sloth Lauszus. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Sloth Lauszus
 Web      :  http://www.lauszus.com
 e-mail   :  lauszus@gmail.com
*/

#ifndef __seriallog_h__
#define __seriallog_h__

#include "HMC5883L.h"
#include "MPU6500.h"
#include "Types.h"

#ifdef __cplusplus
extern "C" {
#endif

enum {
    LOG_NONE = 0,
    LOG_ACC = (1 << 0),
    LOG_GYRO = (1 << 1),
    LOG_MAG = (1 << 2),
    LOG_BARO = (1 << 3),
    LOG_UL = (1 << 4),
    LOG_LIDAR = (1 << 5),
    LOG_MODE = (1 << 6),
    LOG_ANGLE = (1 << 7),
    LOG_HEIGHT = (1 << 8),
    LOG_PID = (1 << 9),
};

void initSerialLog(uint32_t mode);
void serialLogData(angle_t *angle);

#ifdef __cplusplus
}
#endif

#endif
