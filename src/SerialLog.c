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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "AltitudeHold.h"
#include "BMP180.h"
#include "Buzzer.h"
#include "EEPROM.h"
#include "Magnetometer.h"
#include "MPU6500.h"
#include "SerialLog.h"
#include "Time.h"
#include "UART.h"
#include "uartstdio2.h" // Add "UART_BUFFERED2" to preprocessor - it uses a modified version of uartstdio, so it can be used with another UART interface

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/tm4c123gh6pm.h"
#if UART_DEBUG
#include "utils/uartstdio.h" // Add "UART_BUFFERED" to preprocessor - this is used to print to the terminal
#endif

//#define DEBUG_BLUETOOTH_PROTOCOL 0 && UART_DEBUG

static void writeConfig(void);
static void writeHeaders(void);
static uint32_t logMode;

void initSerialLog(uint32_t mode) {
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // Enable the GPIO port containing the pins that will be used.
    SysCtlDelay(2); // Insert a few cycles after enabling the peripheral to allow the clock to be fully activated

    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY; // Unlocks the GPIO_CR register
    GPIO_PORTD_CR_R |= GPIO_PIN_7; // Allow changes to PD7
    GPIO_PORTD_LOCK_R = 0; // Lock register again

    // Configure the GPIO pin muxing for the UART function.
    // This is only necessary if your part supports GPIO pin function muxing.
    // Study the data sheet to see which functions are allocated per pin.
    GPIOPinConfigure(GPIO_PD6_U2RX);
    GPIOPinConfigure(GPIO_PD7_U2TX);

    // Since GPIO D6 and D7 are used for the UART function, they must be
    // configured for use as a peripheral function (instead of GPIO).
    GPIOPinTypeUART(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    UARTStdioConfig2(2, 115200, SysCtlClockGet()); // Mode is set to 8N1 on UART2
    UARTEchoSet2(false);

    while (UARTBusy(UART2_BASE)) {
        // Wait until UART is ready
    }
    
// #if UART_DEBUG
//     while (!UARTRxBytesAvail2()){
//         // Wait for characters
//     }
//     int nbytes = UARTRxBytesAvail2();
//     while (nbytes--)
//         UARTprintf("UART2 %s", UARTgetc2()); // Consume all characters in the buffer
//     UARTFlushTx(false);
// #endif

    // Set log mode
    logMode = mode;

    // Wait for logger to initialise
    delay(1000);

    // Write config
    writeConfig();

    // Write headers
    writeHeaders();

#if UART_DEBUG
    UARTprintf("Config and headers written to Serial Log!\n");
    UARTFlushTx(false);
#endif

}

static void writeConfig(void){

    UARTprintf2("CURRENT CONFIGURATION\n");
    UARTprintf2("---------------------------------\n");
    
    UARTprintf2("PID RollPitch Kp: %d.%03u\n", (int16_t)cfg.pidRollPitchValues.Kp,  (uint16_t)(abs(cfg.pidRollPitchValues.Kp * 1000.0f) % 1000) );
    UARTprintf2("PID RollPitch Ki: %d.%02u\n", (int16_t)cfg.pidRollPitchValues.Ki, (uint16_t)(abs(cfg.pidRollPitchValues.Ki * 100.0f) % 100)  );
    UARTprintf2("PID RollPitch Kd: %d.%05u\n", (int16_t)cfg.pidRollPitchValues.Kd, (uint16_t)(abs(cfg.pidRollPitchValues.Kd * 100000.0f) % 100000)  );
    UARTprintf2("PID RollPitch I-limit: %d.%02u\n", (int16_t)cfg.pidRollPitchValues.integrationLimit, (uint16_t)(abs(cfg.pidRollPitchValues.integrationLimit * 100.0f) % 100) );
    UARTprintf2("PID RollPitch Fc: %d.%01u\n", (int16_t)cfg.pidRollPitchValues.Fc, (uint16_t)(abs(cfg.pidRollPitchValues.Fc * 10.0f) % 10) );

    UARTprintf2("PID Yaw Kp: %d.%03u\n", (int16_t)cfg.pidYawValues.Kp, (uint16_t)(abs(cfg.pidYawValues.Kp * 1000.0f) % 1000) );
    UARTprintf2("PID Yaw Ki: %d.%02u\n", (int16_t)cfg.pidYawValues.Ki, (uint16_t)(abs(cfg.pidYawValues.Ki * 100.0f) % 100) );
    UARTprintf2("PID Yaw Kd: %d.%05u\n", (int16_t)cfg.pidYawValues.Kd, (uint16_t)(abs(cfg.pidYawValues.Kd * 100000.0f) % 100000)  );
    UARTprintf2("PID Yaw I-limit: %d.%02u\n", (int16_t)cfg.pidYawValues.integrationLimit, (uint16_t)(abs(cfg.pidYawValues.integrationLimit * 100.0f) % 100) );
    UARTprintf2("PID Yaw Fc: %d.%01u\n", (int16_t)cfg.pidYawValues.Fc, (uint16_t)(abs(cfg.pidYawValues.Fc * 10.0f) % 10) );

    UARTprintf2("PID Sonar alt hold Kp: %d.%03u\n", (int16_t)cfg.pidSonarAltHoldValues.Kp,  (uint16_t)(abs(cfg.pidSonarAltHoldValues.Kp * 1000.0f) % 1000) );
    UARTprintf2("PID Sonar alt hold Ki: %d.%02u\n", (int16_t)cfg.pidSonarAltHoldValues.Ki, (uint16_t)(abs(cfg.pidSonarAltHoldValues.Ki * 100.0f) % 100) );
    UARTprintf2("PID Sonar alt hold Kd: %d.%05u\n", (int16_t)cfg.pidSonarAltHoldValues.Kd, (uint16_t)(abs(cfg.pidSonarAltHoldValues.Kd * 100000.0f) % 100000) );
    UARTprintf2("PID Sonar alt hold I-limit: %d.%02u\n", (int16_t)cfg.pidSonarAltHoldValues.integrationLimit, (uint16_t)(abs(cfg.pidSonarAltHoldValues.integrationLimit * 100.0f) % 100) );
    UARTprintf2("PID Sonar alt hold Fc: %d.%01u\n", (int16_t)cfg.pidSonarAltHoldValues.Fc, (uint16_t)(abs(cfg.pidSonarAltHoldValues.Fc * 10.0f) % 10) );

    UARTprintf2("PID Baro alt hold Kp: %d.%03u\n", (int16_t)cfg.pidBaroAltHoldValues.Kp,  (uint16_t)(abs(cfg.pidBaroAltHoldValues.Kp * 1000.0f) % 1000) );
    UARTprintf2("PID Baro alt hold Ki: %d.%02u\n", (int16_t)cfg.pidBaroAltHoldValues.Ki, (uint16_t)(abs(cfg.pidBaroAltHoldValues.Ki * 100.0f) % 100) );
    UARTprintf2("PID Baro alt hold Kd: %d.%05u\n", (int16_t)cfg.pidBaroAltHoldValues.Kd, (uint16_t)(abs(cfg.pidBaroAltHoldValues.Kd * 100000.0f) % 100000)  );
    UARTprintf2("PID Baro alt hold I-limit: %d.%02u\n", (int16_t)cfg.pidBaroAltHoldValues.integrationLimit, (uint16_t)(abs(cfg.pidBaroAltHoldValues.integrationLimit * 100.0f) % 100) );
    UARTprintf2("PID Baro alt hold Fc: %d.%01u\n", (int16_t)cfg.pidBaroAltHoldValues.Fc, (uint16_t)(abs(cfg.pidBaroAltHoldValues.Fc * 10.0f) % 10) );

    UARTprintf2("Angle Kp: %d.%02u\n", (int16_t)cfg.angleKp, (uint16_t)(abs(cfg.angleKp * 100.0f) % 100) );
    UARTprintf2("Heading Kp: %d.%02u\n", (int16_t)cfg.headKp, (uint16_t)(abs(cfg.headKp * 100.0f) % 100) );
    UARTprintf2("Max Angle Inclination: %d.%01u\n", (int16_t)cfg.maxAngleInclination,  (uint16_t)(abs(cfg.maxAngleInclination * 10.0f) % 10) ); // Max angle in self level mode
    UARTprintf2("Max Angle Inclination Dist Sensor: %d.%01u\n", (int16_t)cfg.maxAngleInclinationDistSensor, (uint16_t)(abs(cfg.maxAngleInclinationDistSensor * 10.0f) % 10) ); // Max angle when using sonar or LIDAR-Lite v3 in altitude hold mode
    UARTprintf2("Stick scaling RollPitch: %d.%02u\n", (int16_t)cfg.stickScalingRollPitch, (uint16_t)(abs(cfg.stickScalingRollPitch * 100.0f) % 100) );
    UARTprintf2("Stick scaling Yaw: %d.%01u\n", (int16_t)cfg.stickScalingYaw, (uint16_t)(abs(cfg.stickScalingYaw * 10.0f) % 10) );

    UARTprintf2("Acc calibration: %d, %d, %d\n", (int16_t)cfg.accZero.data[0],(int16_t)cfg.accZero.data[1],(int16_t)cfg.accZero.data[2]);
    UARTprintf2("Mag calibration: %d, %d, %d\n", (int16_t)cfg.magZero.data[0],(int16_t)cfg.magZero.data[1],(int16_t)cfg.magZero.data[2]);

    UARTprintf2("---------------------------------\n");
    UARTFlushTx2(false); // Flush TX buffer
    delay(200);
}

static void writeHeaders(void){

    if (logMode && LOG_ANGLE)
        UARTwrite2("roll,pitch,yaw,", 15); // Degrees

    if(logMode && LOG_ACC)
        UARTwrite2("accX,accY,accZ,",15);

    if(logMode && LOG_GYRO)
        UARTwrite2("gyroX,gyroY,gyroZ,",18);
    
     if(logMode && LOG_MAG)
        UARTwrite2("magX,magY,magZ,",15);

#if USE_BARO
    if(logMode && LOG_BARO)
        UARTwrite2("pressure,temperature,absoluteAltitude,groundAltitude,",53);
        //UARTwrite2("alt,zvel,zacc,altLPF,",21); // Millimeters
#endif

#if USE_SONAR //TODO: check if the sonar headers are printed to the log
    if(logMode && LOG_SONAR)
        UARTwrite("sonarDist,",10); // Millimeters
#endif

#if USE_SONAR || USE_LIDAR_LITE
    if(logMode && (LOG_SONAR || LOG_LIDAR))
        UARTwrite("dist,",5); // Millimeters
#endif

    if(logMode != LOG_NONE)
        UARTwrite2("dt",2); // Milliseconds

    // Done writing header
    UARTwrite2("\n",1); // New line
    UARTFlushTx2(false); // Flush TX buffer
    delay(200);
}

void serialLogData(float dt, angle_t *angle,  mpu6500_t *mpu6500, sensor_t *mag, altitude_t *altitude, bmp180_t *baro){
    // TODO: Use UARTwrite2 to make it faster!

    // Write angles
    if (logMode && LOG_ANGLE)
        UARTprintf2("%d.%02u,%d.%02u,%d.%02u,",
                            (int16_t)angle->axis.roll, (uint16_t)(abs(angle->axis.roll*100.0f) % 100),
                            (int16_t)angle->axis.pitch, (uint16_t)(abs(angle->axis.pitch*100.0f) % 100),
                            (int16_t)angle->axis.yaw, (uint16_t)(abs(angle->axis.yaw*100.0f) % 100) );

    if(logMode && LOG_ACC)
        UARTprintf2("%d,%d,%d,",                            
                            mpu6500->acc.axis.X,
                            mpu6500->acc.axis.Y,
                            mpu6500->acc.axis.Z );

    if(logMode && LOG_GYRO)
        UARTprintf2("%d.%03u,%d.%03u,%d.%03u,",
                            (int16_t)mpu6500->gyroRate.axis.roll, (uint16_t)(abs(mpu6500->gyroRate.axis.roll*1000.0f) % 1000),
                            (int16_t)mpu6500->gyroRate.axis.pitch, (uint16_t)(abs(mpu6500->gyroRate.axis.pitch*1000.0f) % 1000),
                            (int16_t)mpu6500->gyroRate.axis.yaw, (uint16_t)(abs(mpu6500->gyroRate.axis.yaw*1000.0f) % 1000) );
    
     if(logMode && LOG_MAG)
        UARTprintf2("%d,%d,%d,",                            
                            mag->axis.X,
                            mag->axis.Y,
                            mag->axis.Z );

#if USE_BARO
    if(logMode && LOG_BARO)
        UARTprintf2("%d,%d,%d,%d,",
                            (int16_t)(baro->pressure),// Pressure in Pa
                            (int16_t)(baro->temperature), // Temperature in 0.1 C
                            (int16_t)(baro->absoluteAltitude), // Absolute altitude in cm
                            (int16_t)(baro->groundAltitude)); // Ground altitude in cm
                            //(int16_t)(altitude->altitude*10.0f),
                            //(int16_t)(altitude->velocity*10.0f),
                            //(int16_t)(altitude->acceleration*10.0f),
                            //(int16_t)(altitude->altitudeLpf*10.0f));
#endif

#if USE_SONAR
    if(logMode && LOG_SONAR)
        UARTprintf2("%d,",altitude->sonarDistance);
#endif

#if USE_SONAR || USE_LIDAR_LITE
    if(logMode && (LOG_SONAR || LOG_LIDAR))
        UARTprintf2("%d,",(int16_t)altitude->distance);
#endif

    if(logMode != LOG_NONE)
        UARTprintf2("%u", (uint16_t)(dt*1e3f)); // Milliseconds

    // Done writing data
    UARTprintf2("\n"); // New line
    UARTFlushTx2(false); // Flush TX buffer
}
