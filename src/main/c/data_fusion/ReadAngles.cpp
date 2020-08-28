#include <time.h>
#include <cmath>
#include "MPU6050_library/MPU6050.h"

#define LOW_PASS_FILTER_GYRO 10 
#define LOW_PASS_FILTER_ACC 20 


MPU6050 mySensor(0x68) ;

float rateX, rateY, rateZ, accX, accY, accZ; 
float rollAcc, pitchAcc, yawAcc; 
float rollGyro, pitchGyro, yawGyro ;
float q1Acc, q2Acc, q3Acc, q4Acc ;
struct timespec tStart, tEnd ; 
float dt ;
int milliSecondDelay = 1 ;
unsigned int microSecondDelay = milliSecondDelay * 1000 ;

float rollGyroArray[LOW_PASS_FILTER_GYRO] ;
float pitchGyroArray[LOW_PASS_FILTER_GYRO] ;
float yawGyroArray[LOW_PASS_FILTER_GYRO] ;
float rollGyroFiltered = 0 , pitchGyroFiltered = 0 , yawGyroFiltered = 0 ;

float rollAccArray[LOW_PASS_FILTER_ACC] ;
float pitchAccArray[LOW_PASS_FILTER_ACC] ;
float yawAccArray[LOW_PASS_FILTER_ACC] ;
float rollAccFiltered = 0 , pitchAccFiltered = 0 , yawAccFiltered = 0 ;

unsigned long int K = 0 ;
int R_Gyro , R_Acc ;

int main() {



	sleep(1) ;

	for (int i=0;i<LOW_PASS_FILTER_GYRO;i++){
		mySensor.getAnglesFromGyro(&rollGyro, &pitchGyro, &yawGyro) ;
		rollGyroArray[i] = rollGyro ;
		pitchGyroArray[i] = pitchGyro ;
		yawGyroArray[i] = yawGyro ;
	}

	for (int i=0;i<LOW_PASS_FILTER_ACC;i++){
		mySensor.getAnglesFromAcc(&accX, &accY, &accZ, &rollAcc, &pitchAcc, &yawAcc) ;
		rollAccArray[i] = rollAcc ;
		pitchAccArray[i] = pitchAcc ;
		yawAccArray[i] = yawAcc ;
	}


	clock_gettime(CLOCK_REALTIME, &tStart);

    while (true){
	K++ ;
	
    mySensor.getAnglesFromGyro(&rollGyro, &pitchGyro, &yawGyro) ;
	mySensor.getAnglesFromAcc(&accX, &accY, &accZ, &rollAcc, &pitchAcc, &yawAcc) ;

	R_Gyro = (K - 1) % LOW_PASS_FILTER_GYRO ;
	R_Acc = (K - 1) % LOW_PASS_FILTER_ACC ;

	rollGyroArray[R_Gyro] = rollGyro ;
	pitchGyroArray[R_Gyro] = pitchGyro ;
	yawGyroArray[R_Gyro] = yawGyro ;

	rollAccArray[R_Acc] = rollAcc ;
	pitchAccArray[R_Acc] = pitchAcc ;
	yawAccArray[R_Acc] = yawAcc ;

	for (int j=1; j<=LOW_PASS_FILTER_GYRO; j++){
		rollGyroFiltered = rollGyroFiltered*(float)(j-1)/(float)j + rollGyroArray[j-1]/j ;
		pitchGyroFiltered = pitchGyroFiltered*(float)(j-1)/(float)j + pitchGyroArray[j-1]/j ;
		yawGyroFiltered = yawGyroFiltered*(float)(j-1)/(float)j + yawGyroArray[j-1]/j ;
	}

	for (int j=1; j<=LOW_PASS_FILTER_ACC; j++){
		rollAccFiltered = rollAccFiltered*(float)(j-1)/(float)j + rollAccArray[j-1]/j ;
		pitchAccFiltered = pitchAccFiltered*(float)(j-1)/(float)j + pitchAccArray[j-1]/j ;
		yawAccFiltered = yawAccFiltered*(float)(j-1)/(float)j + yawAccArray[j-1]/j ;
	}


	std::cout<<"\033[H\033[J";
	std::cout << "-------------------------------------\n" ;
    std::cout << "-------------------------------------\n" ;
	std::cout<<"  Gyro Roll: "<<round(rollGyroFiltered*1000)/1000<<"\n" ;
	std::cout<<"   Acc Roll: "<<round(rollAccFiltered*1000)/1000<<"\n" ;
	std::cout<<"\n" ;
	std::cout<<" Gyro Pitch: "<<round(pitchGyroFiltered*1000)/1000<<"\n" ;
	std::cout<<"  Acc Pitch: "<<round(pitchAccFiltered*1000)/1000<<"\n" ;
	std::cout<<"\n" ;
	std::cout<<"   Gyro Yaw: "<<round(yawGyroFiltered*1000)/1000<<"\n" ;
	std::cout<<"    Acc Yaw: "<<round(yawAccFiltered*1000)/1000<<"\n" ;

	clock_gettime(CLOCK_REALTIME, &tEnd);
	dt = (tEnd.tv_sec - tStart.tv_sec) + (tEnd.tv_nsec - tStart.tv_nsec) / 1e9; 
	clock_gettime(CLOCK_REALTIME, &tStart);
	std::cout<<"\n" ;
	std::cout<<"         dt: "<<round(dt * 1000)<<" milliseconds"<<"\n" ;
	std::cout<<"          K: "<<K<<"\n" ;
    std::cout << "-------------------------------------\n" ;
    std::cout << "-------------------------------------\n" ;
    

	usleep(microSecondDelay);


    }
	return 0;


}







