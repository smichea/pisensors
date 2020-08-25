

#define GYRO_RANGE 0 


#define ACC_RANGE 0 


//Biases (use your own value)
//    Gyro
#define GYRO_BIAS_X -411
#define GYRO_BIAS_Y 127
#define GYRO_BIAS_Z 350
//     Acc
#define ACC_BIAS_X 7808
#define ACC_BIAS_Y -10867
#define ACC_BIAS_Z -14109


#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include "../i2c_library/smbus.h"
#include <cmath>
#include <thread>


#define RAD2DEG 57.29577951308 // 180/pi

#if GYRO_RANGE == 1
	#define GYRO_SENS 65.5
	#define GYRO_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define GYRO_SENS 32.8
	#define GYRO_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define GYRO_SENS 16.4
	#define GYRO_CONFIG 0b00011000
#else 
	#define GYRO_SENS 131.0
	#define GYRO_CONFIG 0b00000000
#endif
#undef GYRO_RANGE


#if GYRO_RANGE == 1
	#define ACC_SENS 8192.0
	#define ACC_CONFIG 0b00001000
#elif GYRO_RANGE == 2
	#define ACC_SENS 4096.0
	#define ACC_CONFIG 0b00010000
#elif GYRO_RANGE == 3
	#define ACC_SENS 2048.0
	#define ACC_CONFIG 0b00011000
#else 
	#define ACC_SENS 16384.0
	#define ACC_CONFIG 0b00000000
#endif
#undef GYRO_RANGE




class MPU6050 {
	private:
		
		float phiAcc, thetaAcc, psiAss ;
		

		int MPU6050_addr;
		int f_dev; //Device file

		float dt; //Loop time


	public:
		MPU6050(int8_t addr);
		void getBias(int totalMeasurement, float *accBias_X, float *accBias_Y, float *accBias_Z, float *gyroBias_X, float *gyroBias_Y, float *gyroBias_Z);
		void getAccelRaw(float *accX, float *accY, float *accZ);
		void getGyroRaw(float *gyroX, float *gyroY, float *gyroZ);
		void getAccel(float *accX, float *accY, float *accZ);
		void getGyro(float *gyroX, float *gyroY, float *gyroZ);
		void getAnglesFromAcc(float *accX, float *accY, float *accZ, float *phiAcc, float *thetaAcc, float *psiAcc) ;
		
};
