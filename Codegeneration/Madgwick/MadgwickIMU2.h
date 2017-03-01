//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================
#ifndef MadgwickIMU2_h
#define MadgwickIMU2_h


//----------------------------------------------------------------------------------------------------
// Include libs
#include "waijung_hwdrvlib.h"

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float beta2;				// algorithm gain
extern volatile float q02, q12, q22, q32;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Function declarations

void MadgwickAHRSupdateIMU2(float in1, float in2, float in3, float in4, float in5, float in6, float *out1, float *out2, float *out3, float *out4);

#endif
//=====================================================================================================
// End of file
//=====================================================================================================
