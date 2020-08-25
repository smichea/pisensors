#include "MPU6050_library/MPU6050.h"
#include <time.h>

MPU6050 device(0x68);


int totalMeasurement = 5000 ; 

int main() {

	float accBias_X, accBias_Y, accBias_Z, gyroBias_X, gyroBias_Y, gyroBias_Z; 

	sleep(1); 

	//Calculate the biases
	std::cout << "Calculating the offsets...\n    Please keep the accelerometer level and still\n    This could take a couple of minutes...";
	device.getBias(totalMeasurement,&accBias_X, &accBias_Y, &accBias_Z, &gyroBias_X, &gyroBias_Y, &gyroBias_Z);
	std::cout << "Gyroscope X,Y,Z: " << gyroBias_X << "," << gyroBias_Y << "," << gyroBias_Z << "\n";
    std::cout << "Accelerometer X,Y,Z: " << accBias_X << "," << accBias_Y << "," << accBias_Z << "\n";

	return 0;
}


