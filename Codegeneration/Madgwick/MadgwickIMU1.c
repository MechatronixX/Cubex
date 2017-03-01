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

#include "MadgwickIMU1.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	500.0f		// sample frequency in Hz
#define betaDef1		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta1 = betaDef1;								// 2 * proportional gain (Kp)
volatile float q01 = 1.0f, q11 = 0.0f, q21 = 0.0f, q31 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt1(float x);

//====================================================================================================
// Functions
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU1(float in1, float in2, float in3, float in4, float in5, float in6, float *out1, float *out2, float *out3, float *out4){
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q01, _2q11, _2q21, _2q31, _4q01, _4q11, _4q21 ,_8q11, _8q21, q01q01, q11q11, q21q21, q31q31;

	// Rate of change of quaternion from in5roscope
	qDot1 = 0.5f * (-q11 * in4 - q21 * in5 - q31 * in6);
	qDot2 = 0.5f * (q01 * in4 + q21 * in6 - q31 * in5);
	qDot3 = 0.5f * (q01 * in5 - q11 * in6 + q31 * in4);
	qDot4 = 0.5f * (q01 * in6 + q11 * in5 - q21 * in4);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((in1 == 0.0f) && (in2 == 0.0f) && (in3 == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt1(in1 * in1 + in2 * in2 + in3 * in3);
		in1 *= recipNorm;
		in2 *= recipNorm;
		in3 *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q01 = 2.0f * q01;
		_2q11 = 2.0f * q11;
		_2q21 = 2.0f * q21;
		_2q31 = 2.0f * q31;
		_4q01 = 4.0f * q01;
		_4q11 = 4.0f * q11;
		_4q21 = 4.0f * q21;
		_8q11 = 8.0f * q11;
		_8q21 = 8.0f * q21;
		q01q01 = q01 * q01;
		q11q11 = q11 * q11;
		q21q21 = q21 * q21;
		q31q31 = q31 * q31;

		// Gradient decent algorithm corrective step
		s0 = _4q01 * q21q21 + _2q21 * in1 + _4q01 * q11q11 - _2q11 * in2;
		s1 = _4q11 * q31q31 - _2q31 * in1 + 4.0f * q01q01 * q11 - _2q01 * in2 - _4q11 + _8q11 * q11q11 + _8q11 * q21q21 + _4q11 * in3;
		s2 = 4.0f * q01q01 * q21 + _2q01 * in1 + _4q21 * q31q31 - _2q31 * in2 - _4q21 + _8q21 * q11q11 + _8q21 * q21q21 + _4q21 * in3;
		s3 = 4.0f * q11q11 * q31 - _2q11 * in1 + 4.0f * q21q21 * q31 - _2q21 * in2;
		recipNorm = invSqrt1(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta1 * s0;
		qDot2 -= beta1 * s1;
		qDot3 -= beta1 * s2;
		qDot4 -= beta1 * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q01 += qDot1 * (1.0f / sampleFreq);
	q11 += qDot2 * (1.0f / sampleFreq);
	q21 += qDot3 * (1.0f / sampleFreq);
	q31 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt1(q01 * q01 + q11 * q11 + q21 * q21 + q31 * q31);
	q01 *= recipNorm;
	q11 *= recipNorm;
	q21 *= recipNorm;
	q31 *= recipNorm;
    
    *out1 = q01;
    *out2 = q11;
    *out3 = q21;
    *out4 = q31;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt1(float x) {
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
