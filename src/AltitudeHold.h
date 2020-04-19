/* Copyright (C) 2015 Kristian Sloth Lauszus. All rights reserved.

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

#if !defined(__altitude_h__) && (USE_SONAR || USE_BARO || USE_LIDAR_LITE)
#define __altitude_h__

#include "BMP180.h"
#include "MPU6500.h"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
#if USE_BARO
    float altitude, velocity, acceleration; // Values are in cm
    float altitudeLpf; // Low-pass filtered altitude estimate
#endif
#if USE_SONAR
    int16_t sonarDistance; // Distance in mm
#endif
#if USE_LIDAR_LITE
    int32_t lidarLiteDistance; // Distance in mm
#endif
#if USE_SONAR || USE_LIDAR_LITE
    float distance; // Fusioned distance from sonar and LIDAR-Lite v3 in mm
#endif
} altitude_t;

void initAltitudeHold(void);
#if USE_BARO
void getAltitude(angle_t *angle, mpu6500_t *mpu6500, altitude_t *altitude, bmp180_t *baro, uint32_t now, float dt);
#else
void getAltitude(angle_t *angle, mpu6500_t *mpu6500, altitude_t *altitude, uint32_t now, float dt);
#endif
float updateAltitudeHold(float aux, altitude_t *altitude, float throttle, uint32_t now, float dt);
void resetAltitudeHold(altitude_t *altitude);

#ifdef __cplusplus
}
#endif

#endif
