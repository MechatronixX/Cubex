//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickIMU3.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	500.0f		// sample frequency in Hz
#define betaDef3		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta3 = betaDef3;								// 2 * proportional gain (Kp)
volatile float q03 = 1.0f, q13 = 0.0f, q23 = 0.0f, q33 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt3(float x);

//====================================================================================================
// Functions
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU3(float in1, float in2, float in3, float in4, float in5, float in6, float *out1, float *out2, float *out3, float *out4){
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q03, _2q13, _2q23, _2q33, _4q03, _4q13, _4q23 ,_8q13, _8q23, q03q03, q13q13, q23q23, q33q33;

	// Rate of change of quaternion from in5roscope
	qDot1 = 0.5f * (-q13 * in4 - q23 * in5 - q33 * in6);
	qDot2 = 0.5f * (q03 * in4 + q23 * in6 - q33 * in5);
	qDot3 = 0.5f * (q03 * in5 - q13 * in6 + q33 * in4);
	qDot4 = 0.5f * (q03 * in6 + q13 * in5 - q23 * in4);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((in1 == 0.0f) && (in2 == 0.0f) && (in3 == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt3(in1 * in1 + in2 * in2 + in3 * in3);
		in1 *= recipNorm;
		in2 *= recipNorm;
		in3 *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q03 = 2.0f * q03;
		_2q13 = 2.0f * q13;
		_2q23 = 2.0f * q23;
		_2q33 = 2.0f * q33;
		_4q03 = 4.0f * q03;
		_4q13 = 4.0f * q13;
		_4q23 = 4.0f * q23;
		_8q13 = 8.0f * q13;
		_8q23 = 8.0f * q23;
		q03q03 = q03 * q03;
		q13q13 = q13 * q13;
		q23q23 = q23 * q23;
		q33q33 = q33 * q33;

		// Gradient decent algorithm corrective step
		s0 = _4q03 * q23q23 + _2q23 * in1 + _4q03 * q13q13 - _2q13 * in2;
		s1 = _4q13 * q33q33 - _2q33 * in1 + 4.0f * q03q03 * q13 - _2q03 * in2 - _4q13 + _8q13 * q13q13 + _8q13 * q23q23 + _4q13 * in3;
		s2 = 4.0f * q03q03 * q23 + _2q03 * in1 + _4q23 * q33q33 - _2q33 * in2 - _4q23 + _8q23 * q13q13 + _8q23 * q23q23 + _4q23 * in3;
		s3 = 4.0f * q13q13 * q33 - _2q13 * in1 + 4.0f * q23q23 * q33 - _2q23 * in2;
		recipNorm = invSqrt3(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta3 * s0;
		qDot2 -= beta3 * s1;
		qDot3 -= beta3 * s2;
		qDot4 -= beta3 * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q03 += qDot1 * (1.0f / sampleFreq);
	q13 += qDot2 * (1.0f / sampleFreq);
	q23 += qDot3 * (1.0f / sampleFreq);
	q33 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt3(q03 * q03 + q13 * q13 + q23 * q23 + q33 * q33);
	q03 *= recipNorm;
	q13 *= recipNorm;
	q23 *= recipNorm;
	q33 *= recipNorm;
    
    *out1 = q03;
    *out2 = q13;
    *out3 = q23;
    *out4 = q33;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt3(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
