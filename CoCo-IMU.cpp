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

#include "CoCo-IMU.h"

IMU::IMU(int imuAddress, int calibrationDelay, LSM6DSM &IMU_REF) : imuAddress(imuAddress), calibrationDelay(calibrationDelay), IMU_REF(IMU_REF) {
	Serial.println("IMU object created.");
}

IMU::~IMU() {
	delete &IMU_REF;
	Serial.println("IMU object destroyed.");
}

void IMU::startIMU() {
	while (IMU_REF.begin(imuAddress) != IMU_SUCCESS) {
		Serial.println("IMU initialization failed.");
		delay(200);
	}

	calibrateGyro();
}

void IMU::calibrateGyro() {
	/* SECTION - Start Gyro Calibration */

	Serial.println("Gyro calibration will start please don't touch the plane");
	delay(calibrationDelay);
	Serial.println("Gyro calibration started");

	int gyroCalibrationIteration = 0;

	for (gyroCalibrationIteration = 0; gyroCalibrationIteration < 2000;
			 gyroCalibrationIteration++) {
		float *allAxesFloatData = new float[7]; // All axes float values
		IMU_REF.readAllAxesFloatData(allAxesFloatData);

		// Add values to the gyroError var
		gyroError.pitch += allAxesFloatData[0];
		gyroError.roll += allAxesFloatData[1];
		gyroError.yaw += allAxesFloatData[2];

		delete[] allAxesFloatData; // Free the memory of allAxes
		delay(1);
	}

	gyroError.pitch /= gyroCalibrationIteration;
	gyroError.roll /= gyroCalibrationIteration;
	gyroError.yaw /= gyroCalibrationIteration;

	Serial.println("Gyro calibration finished");

	/* !SECTION - End Gyro Calibration */
}

void IMU::calculateAndFilterAngles() {
	/* SECTION - Angle Calculations */
	calculatedAngle.pitch = atan(accelG.y / sqrt(pow(accelG.x, 2) + pow(accelG.z, 2))) * 180 / PI;
	calculatedAngle.roll = -atan(accelG.x / sqrt(pow(accelG.y, 2) + pow(accelG.z, 2))) * 180 / PI;

	// filter angle
	kalman1DFilter(kalmanAngle.pitch, kalmanUncertainityAngle.pitch, gyro.pitch, calculatedAngle.pitch);
	kalman1DFilter(kalmanAngle.roll, kalmanUncertainityAngle.roll, gyro.roll, calculatedAngle.roll);
	/* !SECTION - End Angle Calculations */
}

void IMU::kalman1DFilter(float &KalmanState, float &KalmanUncertainity, float KalmanInput, float KalmanMeasurement) {
	KalmanState = KalmanState + 0.004 * KalmanInput;
	KalmanUncertainity = KalmanUncertainity + 0.004 * 4 * 4;
	float KalmanGain = KalmanUncertainity / (KalmanUncertainity + 3 * 3);
	KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
	KalmanUncertainity = (1 - KalmanGain) * KalmanUncertainity;
}

// SECTION - Clamp the gyro data
float gyroMinMax(float data, float errorData) {
	return ((data - errorData) > 1 || (data - errorData < -1)) ? data - errorData : 0;
}
/* !SECTION - End Clamp the gyro data */

void IMU::gatherData() {
	/* SECTION - Data Gathering*/
	float *allAxesFloatData = new float[7]; // All axes float values
	IMU_REF.readAllAxesFloatData(allAxesFloatData);
	gyro.pitch = gyroMinMax(allAxesFloatData[0], gyroError.pitch);
	gyro.roll = gyroMinMax(allAxesFloatData[1], gyroError.roll);
	gyro.yaw = gyroMinMax(allAxesFloatData[2], gyroError.yaw);

	accelG.x = allAxesFloatData[3];
	accelG.y = allAxesFloatData[4];
	accelG.z = allAxesFloatData[5];

	accelMps2.x = accelG.x * GRAVITAIONAL_ACCELERATION;
	accelMps2.y = accelG.y * GRAVITAIONAL_ACCELERATION;
	accelMps2.z = accelG.z * GRAVITAIONAL_ACCELERATION;

	delete[] allAxesFloatData; // Free the memory of allAxes
	/* !SECTION - End Data Gathering */

	calculateAndFilterAngles(); //
}
