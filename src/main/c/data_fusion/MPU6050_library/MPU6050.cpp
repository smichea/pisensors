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
	

	i2c_smbus_write_byte_data(f_dev, 0x06, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x07, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x08, 0b00000000),
	i2c_smbus_write_byte_data(f_dev, 0x09, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x0A, 0b00000000), i2c_smbus_write_byte_data(f_dev, 0x0B, 0b00000000),
	i2c_smbus_write_byte_data(f_dev, 0x00, 0b10000001), i2c_smbus_write_byte_data(f_dev, 0x01, 0b00000001), i2c_smbus_write_byte_data(f_dev, 0x02, 0b10000001);
	
	
	std::thread(&MPU6050::gyroIntegrating, this).detach();

}

void MPU6050::getBias(int totalMeasurement, float *accBias_X, float *accBias_Y, float *accBias_Z, float *gyroBias_X, float *gyroBias_Y, float *gyroBias_Z) {
	float gyroBias[3]; 
	float accBias[3];

	*gyroBias_X = 0, *gyroBias_Y = 0, *gyroBias_Z = 0; 
	*accBias_X = 0, *accBias_Y = 0, *accBias_Z = 0; 
	for (float i = 1; i <= totalMeasurement; i++) { 

		getGyroRaw(&gyroBias[0], &gyroBias[1], &gyroBias[2]); //Raw gyroscope values
		*gyroBias_X = *gyroBias_X*(i-1)/i + gyroBias[0]/i ; 
		*gyroBias_Y = *gyroBias_Y*(i-1)/i + gyroBias[1]/i ;  
		*gyroBias_Z = *gyroBias_Z*(i-1)/i + gyroBias[2]/i ; 

		getAccRaw(&accBias[0], &accBias[1], &accBias[2]); //Raw accelerometer values
		*accBias_X = *accBias_X*(i-1)/i + accBias[0]/i ;
		*accBias_Y = *accBias_Y*(i-1)/i + accBias[1]/i ;
		*accBias_Z = *accBias_Z*(i-1)/i + accBias[2]/i ; 
		
		std::cout<<"\033[H\033[J";
    	std::cout << "-------------------------------------\n" ;
    	std::cout << "-------------------------------------\n" ;
		std::cout <<"Measurement number "<<i<<" from totally "<<totalMeasurement<<" measurements:\n";
		std::cout <<"\n";
		std::cout <<"Bias Gyro X: "<<round(*gyroBias_X)<<"\n";
		std::cout <<"Bias Gyro Y: "<<round(*gyroBias_Y)<<"\n";
		std::cout <<"Bias Gyro Z: "<<round(*gyroBias_Z)<<"\n";

		std::cout <<"\n";

		std::cout <<"Bias Acc X: "<<round(*accBias_X)<<"\n";
		std::cout <<"Bias Acc Y: "<<round(*accBias_Y)<<"\n";
		std::cout <<"Bias Acc Z: "<<round(*accBias_Z - ACC_SENS)<<"\n";
		std::cout << "-------------------------------------\n" ;
   	    std::cout << "-------------------------------------\n" ;

	}
	*accBias_Z = *accBias_Z - ACC_SENS ;
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

void MPU6050::getAccRaw(float *accX, float *accY, float *accZ) {
	int16_t acc_x = i2c_smbus_read_byte_data(f_dev, 0x3b) << 8 | i2c_smbus_read_byte_data(f_dev, 0x3c); //Read Acc X registers
	int16_t acc_y = i2c_smbus_read_byte_data(f_dev, 0x3d) << 8 | i2c_smbus_read_byte_data(f_dev, 0x3e); //Read Acc Y registers
	int16_t acc_z = i2c_smbus_read_byte_data(f_dev, 0x3f) << 8 | i2c_smbus_read_byte_data(f_dev, 0x40); //Read Acc Z registers
	*accX = (float)acc_x;
	*accY = (float)acc_y;
	*accZ = (float)acc_z;
}

void MPU6050::getAcc(float *accX, float *accY, float *accZ) {
	getAccRaw(accX, accY, accZ); 
	*accX = round((*accX - ACC_BIAS_X) * 1000.0 / ACC_SENS) / 1000.0; ////Bias compensation and divide by the accelerometer sensetivity
	*accY = round((*accY - ACC_BIAS_Y) * 1000.0 / ACC_SENS) / 1000.0;
	*accZ = round((*accZ - ACC_BIAS_Z) * 1000.0 / ACC_SENS) / 1000.0;
}


void MPU6050::getAnglesFromAcc(float *accX, float *accY, float *accZ, float *rollAcc, float *pitchAcc, float *yawAcc) {
	getAcc(accX, accY, accZ);
	*rollAcc = -atan2(-*accY,*accZ) * RAD2DEG ;
	*pitchAcc = -asin(*accX) * RAD2DEG ;
	*yawAcc = 0 * RAD2DEG ;


}

void MPU6050::EulerAngle2Qauternion(float *roll, float *pitch, float *yaw, float *Q1, float *Q2, float *Q3, float *Q4){
	float rollRad = *roll / RAD2DEG ;
	float pitchRad = *pitch / RAD2DEG ;
	float yawRad = *yaw / RAD2DEG ;
	*Q1 = sin(rollRad/2)*cos(pitchRad/2)*cos(yawRad/2) - cos(rollRad/2)*sin(pitchRad/2)*sin(yawRad/2) ;
	*Q2 = cos(rollRad/2)*sin(pitchRad/2)*cos(yawRad/2) + sin(rollRad/2)*cos(pitchRad/2)*sin(yawRad/2) ;
	*Q3 = cos(rollRad/2)*cos(pitchRad/2)*sin(yawRad/2) - sin(rollRad/2)*sin(pitchRad/2)*cos(yawRad/2) ;
	*Q4 = cos(rollRad/2)*cos(pitchRad/2)*cos(yawRad/2) + sin(rollRad/2)*sin(pitchRad/2)*sin(yawRad/2) ;
	

}


void MPU6050::Quaternion2EulerAngle(float *roll, float *pitch, float *yaw, float *Q1, float *Q2, float *Q3, float *Q4){
	
	float rollRad, pitchRad, yawRad ;

	rollRad = atan2( 2*( (*Q4) * (*Q1) + (*Q2) * (*Q3) ) , 1 - 2*( (*Q1) * (*Q1) + (*Q2) * (*Q2) ) ) ;
	pitchRad = asin( 2*( (*Q4) * (*Q2) - (*Q3) * (*Q1) ) ) ;
	yawRad = atan2( 2*( (*Q4) * (*Q3) + (*Q1) * (*Q2) ) , 1 - 2*( (*Q2) * (*Q2) + (*Q3) * (*Q3) ) ) ;

	*roll = rollRad * RAD2DEG ;
	*pitch = pitchRad * RAD2DEG ;
	*yaw = yawRad * RAD2DEG ; 	

}


void MPU6050::dQ(float *rateX, float *rateY, float *rateZ, float *Q1, float *Q2, float *Q3, float *Q4, float *dQ1, float *dQ2, float *dQ3, float *dQ4){

	float rateXRad = *rateX / RAD2DEG ;
	float rateYRad = *rateY / RAD2DEG ;
	float rateZRad = *rateZ / RAD2DEG ;

	float q1 = *Q1 ;
	float q2 = *Q2 ;
	float q3 = *Q3 ; 
	float q4 = *Q4 ;
	*dQ1 = 0.5 * ( 0 + rateZRad*q2 - rateYRad*q3 + rateXRad*q4 ) ;
	*dQ2 = 0.5 * ( 0 - rateZRad*q1 + rateXRad*q3 + rateYRad*q4 ) ;
	*dQ3 = 0.5 * ( 0 + rateYRad*q1 - rateXRad*q2 + rateZRad*q4 ) ;
	*dQ4 = 0.5 * ( 0 - rateXRad*q1 - rateYRad*q2 - rateZRad*q3 ) ;

}


void MPU6050::getAnglesFromGyro(float *roll, float *pitch, float *yaw){
		*roll = rollGyroMain ;
		*pitch = pitchGyroMain ;
		*yaw = yawGyroMain ;
}


void MPU6050::gyroIntegrating(){  //runs continuously


	int initAngleLoop = 100 ;
	float normQGyro ;

	for (int i=1;i<initAngleLoop;i++){
		getAnglesFromAcc(&accXmain, &accYmain, &accZmain, &rollAccMain, &pitchAccMain, &yawAccMain);
		initRoll = initRoll + rollAccMain ;
		initPitch = initPitch + pitchAccMain ;
	}
	initRoll = round(initRoll * 1000 / initAngleLoop ) / 1000 ; 
	initPitch = round(initPitch * 1000 / initAngleLoop ) / 1000 ;

	EulerAngle2Qauternion(&initRoll, &initPitch, &initYaw, &q1Gyro, &q2Gyro, &q3Gyro, &q4Gyro) ;

	clock_gettime(CLOCK_REALTIME, &tStart);
	while(true) {

		getGyro(&gyroXmain, &gyroYmain, &gyroZmain) ;
		dQ(&gyroXmain, &gyroYmain, &gyroZmain, &q1Gyro, &q2Gyro, &q3Gyro, &q4Gyro, &dq1Gyro, &dq2Gyro, &dq3Gyro, &dq4Gyro) ;

		clock_gettime(CLOCK_REALTIME, &tEnd);
		dt = (tEnd.tv_sec - tStart.tv_sec) + (tEnd.tv_nsec - tStart.tv_nsec) / 1e9; 
		clock_gettime(CLOCK_REALTIME, &tStart);
		q1Gyro = q1Gyro + dt*dq1Gyro ;
		q2Gyro = q2Gyro + dt*dq2Gyro ;
		q3Gyro = q3Gyro + dt*dq3Gyro ;
		q4Gyro = q4Gyro + dt*dq4Gyro ;

		normQGyro = sqrt(q1Gyro*q1Gyro+q2Gyro*q2Gyro+q3Gyro*q3Gyro+q4Gyro*q4Gyro) ;
		q1Gyro = q1Gyro / normQGyro ;
		q2Gyro = q2Gyro / normQGyro ;
		q3Gyro = q3Gyro / normQGyro ;
		q4Gyro = q4Gyro / normQGyro ;

		Quaternion2EulerAngle(&rollGyroMain, &pitchGyroMain, &yawGyroMain, &q1Gyro, &q2Gyro, &q3Gyro, &q4Gyro) ;
		

		//std::cout<<""<<sqrt(q1Gyro*q1Gyro+q2Gyro*q2Gyro+q3Gyro*q3Gyro+q4Gyro*q4Gyro)<<"\n" ;
		//printf("\033[H\033[J");
		//std::cout<<""<<rollGyroMain<<"\n" ;
		//std::cout<<""<<pitchGyroMain<<"\n" ;
		//std::cout<<""<<yawGyroMain<<"\n" ;
		//std::cout<<""<<dt*1000<<"\n" ;
    	//std::cout << "-------------------------------------\n" ;
    	//std::cout << "-------------------------------------\n" ;

	}

}
