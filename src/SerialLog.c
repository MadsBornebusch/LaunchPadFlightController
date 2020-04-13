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

#include "SerialLog.h"
#include "Buzzer.h"
#include "EEPROM.h"
#include "Magnetometer.h"
#include "MPU6500.h"
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
    UARTwrite2("Test UART2\n", 12);
    UARTFlushTx2(false);

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
    
    UARTprintf2("PID RollPitch Kp: %.3f\n", (double)cfg.pidRollPitchValues.Kp );
    UARTprintf2("PID RollPitch Ki: %.2f\n", (double)cfg.pidRollPitchValues.Ki );
    UARTprintf2("PID RollPitch Kd: %.5f\n", (double)cfg.pidRollPitchValues.Kd );
    UARTprintf2("PID RollPitch I-limit: %.2f\n", (double)cfg.pidRollPitchValues.integrationLimit );
    UARTprintf2("PID RollPitch Fc: %.1f\n", (double)cfg.pidRollPitchValues.Fc );

    UARTprintf2("PID Yaw Kp: %.3f\n", (double)cfg.pidYawValues.Kp );
    UARTprintf2("PID Yaw Ki: %.2f\n", (double)cfg.pidYawValues.Ki );
    UARTprintf2("PID Yaw Kd: %.5f\n", (double)cfg.pidYawValues.Kd );
    UARTprintf2("PID Yaw I-limit: %.2f\n", (double)cfg.pidYawValues.integrationLimit );
    UARTprintf2("PID Yaw Fc: %.1f\n", (double)cfg.pidYawValues.Fc );

    UARTprintf2("PID Sonar alt hold Kp: %.3f\n", (double)cfg.pidSonarAltHoldValues.Kp );
    UARTprintf2("PID Sonar alt hold Ki: %.2f\n", (double)cfg.pidSonarAltHoldValues.Ki );
    UARTprintf2("PID Sonar alt hold Kd: %.5f\n", (double)cfg.pidSonarAltHoldValues.Kd );
    UARTprintf2("PID Sonar alt hold I-limit: %.2f\n", (double)cfg.pidSonarAltHoldValues.integrationLimit );
    UARTprintf2("PID Sonar alt hold Fc: %.1f\n", (double)cfg.pidSonarAltHoldValues.Fc );

    UARTprintf2("PID Baro alt hold Kp: %.3f\n", (double)cfg.pidBaroAltHoldValues.Kp );
    UARTprintf2("PID Baro alt hold Ki: %.2f\n", (double)cfg.pidBaroAltHoldValues.Ki );
    UARTprintf2("PID Baro alt hold Kd: %.5f\n", (double)cfg.pidBaroAltHoldValues.Kd );
    UARTprintf2("PID Baro alt hold I-limit: %.2f\n", (double)cfg.pidBaroAltHoldValues.integrationLimit );
    UARTprintf2("PID Baro alt hold Fc: %.1f\n", (double)cfg.pidBaroAltHoldValues.Fc );

    UARTprintf2("Angle Kp: %.2f\n", (double)cfg.angleKp );
    UARTprintf2("Heading Kp: %.2f\n", (double)cfg.headKp );
    UARTprintf2("Max Angle Inclination: %.1f\n", (double)cfg.maxAngleInclination ); // Max angle in self level mode
    UARTprintf2("Max Angle Inclination Dist Sensor: %.1f\n", (double)cfg.maxAngleInclinationDistSensor ); // Max angle when using sonar or LIDAR-Lite v3 in altitude hold mode
    UARTprintf2("Stick scaling RollPitch: %.2f\n", (double)cfg.stickScalingRollPitch );
    UARTprintf2("Stick scaling Yaw: %.1f\n", (double)cfg.stickScalingYaw );

    UARTprintf2("Stick scaling Yaw: %.1f\n", (double)cfg.stickScalingYaw );

    UARTprintf2("Acc calibration: %d, %d, %d\n", (int16_t)cfg.accZero.data[0],(int16_t)cfg.accZero.data[1],(int16_t)cfg.accZero.data[2]);
    UARTprintf2("Mag calibration: %d, %d, %d\n", (int16_t)cfg.magZero.data[0],(int16_t)cfg.magZero.data[1],(int16_t)cfg.magZero.data[2]);

    UARTprintf2("---------------------------------\n");
    UARTFlushTx2(false); // Flush TX buffer
}

static void writeHeaders(void){

    if (logMode && LOG_ANGLE)
        UARTprintf2("roll,pitch,yaw");

    // Done writing header
    UARTprintf2("\n"); // Print carriage return and line feed
    UARTFlushTx2(false); // Flush TX buffer
}

void serialLogData(angle_t *angle){
    // TODO: Use UARTwrite2 to make it faster!
    
    // Write angles
    if (logMode && LOG_ANGLE)
        UARTprintf2("%d,%d,%d",(int16_t)angle->axis.roll, (int16_t)angle->axis.pitch, (int16_t)angle->axis.yaw);

    // Done writing data
    UARTprintf2("\n"); // Print carriage return and line feed
    UARTFlushTx2(false); // Flush TX buffer
}
