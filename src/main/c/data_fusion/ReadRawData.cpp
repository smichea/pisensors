#include <time.h>

#include <cmath>

#include "MPU6050_library/MPU6050.h"



MPU6050 mySensor(0x68) ;


int main() {

	float rateX, rateY, rateZ, accX, accY, accZ; 
		
	
	float dt ;

	int dtMilli ;

	struct timespec tStart,tEnd ;

	sleep(1); 

	clock_gettime(CLOCK_REALTIME, &tStart); 

	while (true){
		
		printf("\033[H\033[J");
		
		mySensor.getGyro(&rateX, &rateY, &rateZ) ;
		mySensor.getAcc(&accX, &accY, &accZ) ;
	
		std::cout << "-------------------------------------\n" ;
		std::cout << "-------------------------------------\n" ;
		std::cout<<"\n" ;
		std::cout << "Rate X: " << rateX << " deg/sec\n" ;
		std::cout << "Rate Y: " << rateY << " deg/sec\n" ; 
		std::cout << "Rate Z: " << rateZ << " deg/sec\n" ;
		std::cout<<"\n" ;
		std::cout << " Acc X: " << accX << " g\n" ;
		std::cout << " Acc Y: " << accY << " g\n" ;
		std::cout << " Acc Z: " << accZ << " g\n" ;
		std::cout<<"\n" ;
		
		clock_gettime(CLOCK_REALTIME, &tEnd);
		dt = (tEnd.tv_sec - tStart.tv_sec) + (tEnd.tv_nsec - tStart.tv_nsec) / 1e9; 
		dtMilli = round(dt * 1000) ;

		std::cout << "    dt: " << dtMilli << " milliseconds\n" ;
		std::cout<<"\n" ;
		std::cout << "-------------------------------------\n" ;
		std::cout << "-------------------------------------\n" ;

		clock_gettime(CLOCK_REALTIME, &tStart); 

	}

	return 0;

}





