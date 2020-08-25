#include <time.h>

#include <cmath>

#include "MPU6050_library/MPU6050.h"



MPU6050 mySensor(0x68) ;



MPU6050 sensorMPU6050(0x68);



int main() {

	float rateX, rateY, rateZ, accX, accY, accZ; //Variables to store the accel, gyro and angle values
		
	float phiAcc, thetaAcc, psiAcc; 
	
	float dt ;

	int dtMilli ;

	struct timespec tS,tF ;

	sleep(1); //Wait for the MPU6050 to stabilize

	clock_gettime(CLOCK_REALTIME, &tS); 

	while (true){
		
		printf("\033[H\033[J");
		
		sensorMPU6050.getGyro(&rateX, &rateY, &rateZ) ;
		sensorMPU6050.getAccel(&accX, &accY, &accZ) ;
		sensorMPU6050.getAnglesFromAcc(&accX, &accY, &accZ, &phiAcc, &thetaAcc, &psiAcc) ;
	
		std::cout << "-------------------------------------\n" ;
		std::cout << "-------------------------------------\n" ;
		std::cout << "Rate X: " << rateX << " deg/sec\n" ;
		std::cout << "Rate Y: " << rateY << " deg/sec\n" ; 
		std::cout << "Rate Z: " << rateZ << " deg/sec\n" ;
	
		std::cout << "Acc X: " << accX << " g\n" ;
		std::cout << "Acc Y: " << accY << " g\n" ;
		std::cout << "Acc Z: " << accZ << " g\n" ;
	
		std::cout << "\n" ;
		std::cout << "Phi Acc: " << phiAcc << " deg\n" ;
		std::cout << "Theta Acc: " << thetaAcc << " deg\n" ;
		std::cout << "Psi Acc: " << psiAcc << " deg\n" ;
		
		clock_gettime(CLOCK_REALTIME, &tF);
		dt = (tF.tv_sec - tS.tv_sec) + (tF.tv_nsec - tS.tv_nsec) / 1e9; //Calculate new dt
		dtMilli = round(dt * 1000) ;

		std::cout << "Loop Time: " << dtMilli << " milli-second\n" ;
		std::cout << "-------------------------------------\n" ;
		std::cout << "-------------------------------------\n" ;

		clock_gettime(CLOCK_REALTIME, &tS); 

	}

	return 0;

}





