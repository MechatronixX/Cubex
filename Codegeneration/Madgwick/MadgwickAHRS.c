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

#include "MadgwickAHRS.h"
#include <math.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	500.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q02 = 1.0f, q12 = 0.0f, q22 = 0.0f, q32 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions
//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float in1, float in2, float in3, float in4, float in5, float in6, float *out1, float *out2, float *out3, float *out4){
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q02, _2q12, _2q22, _2q32, _4q02, _4q12, _4q22 ,_8q12, _8q22, q02q02, q12q12, q22q22, q32q32;

	// Rate of change of quaternion from in5roscope
	qDot1 = 0.5f * (-q12 * in4 - q22 * in5 - q32 * in6);
	qDot2 = 0.5f * (q02 * in4 + q22 * in6 - q32 * in5);
	qDot3 = 0.5f * (q02 * in5 - q12 * in6 + q32 * in4);
	qDot4 = 0.5f * (q02 * in6 + q12 * in5 - q22 * in4);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((in1 == 0.0f) && (in2 == 0.0f) && (in3 == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(in1 * in1 + in2 * in2 + in3 * in3);
		in1 *= recipNorm;
		in2 *= recipNorm;
		in3 *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q02 = 2.0f * q02;
		_2q12 = 2.0f * q12;
		_2q22 = 2.0f * q22;
		_2q32 = 2.0f * q32;
		_4q02 = 4.0f * q02;
		_4q12 = 4.0f * q12;
		_4q22 = 4.0f * q22;
		_8q12 = 8.0f * q12;
		_8q22 = 8.0f * q22;
		q02q02 = q02 * q02;
		q12q12 = q12 * q12;
		q22q22 = q22 * q22;
		q32q32 = q32 * q32;

		// Gradient decent algorithm corrective step
		s0 = _4q02 * q22q22 + _2q22 * in1 + _4q02 * q12q12 - _2q12 * in2;
		s1 = _4q12 * q32q32 - _2q32 * in1 + 4.0f * q02q02 * q12 - _2q02 * in2 - _4q12 + _8q12 * q12q12 + _8q12 * q22q22 + _4q12 * in3;
		s2 = 4.0f * q02q02 * q22 + _2q02 * in1 + _4q22 * q32q32 - _2q32 * in2 - _4q22 + _8q22 * q12q12 + _8q22 * q22q22 + _4q22 * in3;
		s3 = 4.0f * q12q12 * q32 - _2q12 * in1 + 4.0f * q22q22 * q32 - _2q22 * in2;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q02 += qDot1 * (1.0f / sampleFreq);
	q12 += qDot2 * (1.0f / sampleFreq);
	q22 += qDot3 * (1.0f / sampleFreq);
	q32 += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q02 * q02 + q12 * q12 + q22 * q22 + q32 * q32);
	q02 *= recipNorm;
	q12 *= recipNorm;
	q22 *= recipNorm;
	q32 *= recipNorm;
    
    *out1 = q02;
    *out2 = q12;
    *out3 = q22;
    *out4 = q32;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
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
