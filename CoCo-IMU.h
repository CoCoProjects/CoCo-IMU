/************************************************************************
																																				Source
Code Form License Notice
																								-------------------------------------------

	This Source Code Form is subject to the terms of the Mozilla Public
	License, v. 2.0. If a copy of the MPL was not distributed with this
	file, You can obtain one at http://mozilla.org/MPL/2.0/.

If it is not possible or desirable to put the notice in a particular
file, then You may include the notice in a location (such as a LICENSE
file in a relevant directory) where a recipient would be likely to look
for such a notice.
*************************************************************************/

#ifndef COCO_IMU_H
#define COCO_IMU_H

#include <Arduino.h>
#include <Deneyap_6EksenAtaletselOlcumBirimi.h>

//* CONSTANTS
#define GRAVITAIONAL_ACCELERATION 9.80665

struct TwoDimentionalCartesian {
	float x, y;
};

struct ThreeDimentionalCartesian {
	float x, y, z;
};

struct TwoDimentionalAngular {
	float pitch, roll;
};

struct ThreeDimentionalAngular {
	float pitch, roll, yaw;
};

class IMU {
	private:
	int imuAddress, calibrationDelay;
	LSM6DSM &IMU_REF;

	// Data vars
	ThreeDimentionalAngular gyro = {
			.pitch = 0,
			.roll = 0,
			.yaw = 0};
	ThreeDimentionalAngular gyroError = {
			.pitch = 0,
			.roll = 0,
			.yaw = 0};
    ThreeDimentionalCartesian accelG = {
            .x = 0,
            .y = 0,
            .z = 0};
    ThreeDimentionalCartesian accelMps2 = {
            .x = 0,
            .y = 0,
            .z = 0};
    ThreeDimentionalAngular calculatedAngle = {
            .pitch = 0,
            .roll = 0,
            .yaw = 0};
    TwoDimentionalAngular kalmanAngle = {
            .pitch = 0,
            .roll = 0};
    TwoDimentionalAngular kalmanUncertainityAngle = {
            .pitch = 0,
            .roll = 0};

	void calculateAngles();
	void kalman1DFilter(float &, float &, float, float);
    float gyroMinMax(float, float);

	public:

	IMU(int, int, LSM6DSM &);
	void startIMU();
	void calibrateGyro();
	void gatherData();
	void plotValues();
	~IMU();
};

#endif