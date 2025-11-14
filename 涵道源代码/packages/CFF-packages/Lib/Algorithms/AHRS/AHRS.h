/*!
 * @file AHRS
 *
 * @mainpage AHRS
 *
 * @section intro_sec Introduction
 *
 * This library lets you take an accelerometer, gyroscope and magnetometer
 * and combine the data to create orientation data.
 *
 * Options are Mahony (lowest memory/computation),
 * Madgwick (fair memory/computation).
 *
 * While in theory these can run on an Arduino UNO/Atmega328P we really
 * recommend a SAMD21 or better. Having single-instruction floating point
 * multiply and plenty of RAM will help a lot!
 */

#ifndef __AHRS_H_
#define __AHRS_H_

#include <AHRS_Madgwick.h>
#include <AHRS_Mahony.h>

#endif // __AHRS_H_
