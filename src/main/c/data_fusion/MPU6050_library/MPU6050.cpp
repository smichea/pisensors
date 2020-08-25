//Includes
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <cmath>
#include <thread>
#include "MPU6050.h"

MPU6050::MPU6050(int8_t addr) {
	int status;

	MPU6050_addr = addr;

	f_dev = open("/dev/i2c-1", O_RDWR); //Open the I2C device file
	if (f_dev < 0) { 
		std::cout << "ERR (MPU6050.cpp:MPU6050()): Failed to open /dev/i2c-1. Please check that I2C is enabled with raspi-config\n"; 
	}

	status = ioctl(f_dev, I2C_SLAVE, MPU6050_addr);
	if (status < 0) {
		std::cout << "ERR (MPU6050.cpp:MPU6050()): Could not get I2C bus with " << addr << " address. Please confirm that this address is correct\n";
	}

	i2c_smbus_write_byte_data(f_dev, 0x6b, 0b00000000); 

	i2c_smbus_write_byte_data(f_dev, 0x1a, 0b00000011); //Set DLPF (low pass filter) to 44Hz

	i2c_smbus_write_byte_data(f_dev, 0x19, 0b00000100); //Set sample rate divider (to 200Hz)

	i2c_smbus_write_byte_data(f_dev, 0x1b, GYRO_CONFIG); //Configure gyroscope settings 

	i2c_smbus_write_byte_data(f_dev, 0x1c, ACC_CONFIG); //Configure accelerometer settings 

	i2c_smbus_write_byte_data(f_dev, 0x06, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x07, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x08, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x09, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x0A, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x0B, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x00, 0b10000001), i2c_smbus_write_byte_data(f_dev, 0x01, 0b00000001), i2c_smbus_write_byte_data(f_dev, 0x02, 0b10000001);

}

void MPU6050::getBias(int totalMeasurement, float *accBias_X, float *accBias_Y, float *accBias_Z, float *gyroBias_X, float *gyroBias_Y, float *gyroBias_Z) {
	float gyroBias[3]; 
	float accBias[3];

	*gyroBias_X = 0, *gyroBias_Y = 0, *gyroBias_Z = 0; 
	*accBias_X = 0, *accBias_Y = 0, *accBias_Z = 0; 
	for (int i = 0; i < totalMeasurement; i++) { 
		getGyroRaw(&gyroBias[0], &gyroBias[1], &gyroBias[2]); //Raw gyroscope values
		*gyroBias_X = *gyroBias_X + gyroBias[0], *gyroBias_Y = *gyroBias_Y + gyroBias[1], *gyroBias_Z = *gyroBias_Z + gyroBias[2]; 

		getAccelRaw(&accBias[0], &accBias[1], &accBias[2]); //Raw accelerometer values
		*accBias_X = *accBias_X + accBias[0], *accBias_Y = *accBias_Y + accBias[1], *accBias_Z = *accBias_Z + accBias[2]; 
		
		std::cout <<"Measurement number "<<i<<" from totally "<<totalMeasurement<<" measurements.\n";
	}

	*gyroBias_X = *gyroBias_X / totalMeasurement, *gyroBias_Y = *gyroBias_Y / totalMeasurement, *gyroBias_Z = *gyroBias_Z / totalMeasurement; //Divide by number of loops 
	*accBias_X = *accBias_X / totalMeasurement, *accBias_Y = *accBias_Y / totalMeasurement, *accBias_Z = *accBias_Z / totalMeasurement;

	*accBias_Z = *accBias_Z - ACC_SENS; 
}

void MPU6050::getGyroRaw(float *gyroX, float *gyroY, float *gyroZ) {
	int16_t gyro_x = i2c_smbus_read_byte_data(f_dev, 0x43) << 8 | i2c_smbus_read_byte_data(f_dev, 0x44); //Read Gyro X registers
	int16_t gyro_y = i2c_smbus_read_byte_data(f_dev, 0x45) << 8 | i2c_smbus_read_byte_data(f_dev, 0x46); //Read Gyro Y registers
	int16_t gyro_z = i2c_smbus_read_byte_data(f_dev, 0x47) << 8 | i2c_smbus_read_byte_data(f_dev, 0x48); //Read Gyro Z registers
	*gyroX = (float)gyro_x; //Gyro valus on X axis
	*gyroY = (float)gyro_y; //Gyro valus on Y axis
	*gyroZ = (float)gyro_z; //Gyro valus on Z axis
}

void MPU6050::getGyro(float *gyroX, float *gyroY, float *gyroZ) {
	getGyroRaw(gyroX, gyroY, gyroZ); 
	*gyroX = round((*gyroX - GYRO_BIAS_X) * 1000.0 / GYRO_SENS) / 1000.0; //Bias compensation and divide by the gyroscope sensetivity
	*gyroY = round((*gyroY - GYRO_BIAS_Y) * 1000.0 / GYRO_SENS) / 1000.0;
	*gyroZ = round((*gyroZ - GYRO_BIAS_Z) * 1000.0 / GYRO_SENS) / 1000.0;
}

void MPU6050::getAccelRaw(float *accX, float *accY, float *accZ) {
	int16_t acc_x = i2c_smbus_read_byte_data(f_dev, 0x3b) << 8 | i2c_smbus_read_byte_data(f_dev, 0x3c); //Read Acc X registers
	int16_t acc_y = i2c_smbus_read_byte_data(f_dev, 0x3d) << 8 | i2c_smbus_read_byte_data(f_dev, 0x3e); //Read Acc Y registers
	int16_t acc_z = i2c_smbus_read_byte_data(f_dev, 0x3f) << 8 | i2c_smbus_read_byte_data(f_dev, 0x40); //Read Acc Z registers
	*accX = (float)acc_x;
	*accY = (float)acc_y;
	*accZ = (float)acc_z;
}

void MPU6050::getAccel(float *accX, float *accY, float *accZ) {
	getAccelRaw(accX, accY, accZ); 
	*accX = round((*accX - ACC_BIAS_X) * 1000.0 / ACC_SENS) / 1000.0; ////Bias compensation and divide by the accelerometer sensetivity
	*accY = round((*accY - ACC_BIAS_Y) * 1000.0 / ACC_SENS) / 1000.0;
	*accZ = round((*accZ - ACC_BIAS_Z) * 1000.0 / ACC_SENS) / 1000.0;
}


void MPU6050::getAnglesFromAcc(float *accX, float *accY, float *accZ, float *phiAcc, float *thetaAcc, float *psiAcc) {
	getAccel(accX, accY, accZ);
	*phiAcc = atan2(-*accY,*accZ) * RAD2DEG ;
	*thetaAcc = asin(*accX) * RAD2DEG ;
	*psiAcc = 0 * RAD2DEG ;
	
	
}

