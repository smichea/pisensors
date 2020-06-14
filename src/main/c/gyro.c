#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <i2c/smbus.h>

#define Device_Address 0x68	
#define PWR_MGMT_1   0x6B
#define SMPLRT_DIV   0x19
#define CONFIG       0x1A
#define GYRO_CONFIG  0x1B
#define INT_ENABLE   0x38
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H  0x43
#define GYRO_YOUT_H  0x45
#define GYRO_ZOUT_H  0x47

int read_reg(int fd, int reg)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access (fd, I2C_SMBUS_READ, reg, I2C_SMBUS_BYTE_DATA, &data))
    return -1 ;
  else
    return data.byte & 0xFF ;
}

int write_reg (int fd, int reg, int value)
{
  union i2c_smbus_data data ;
  data.byte = value ;
  return i2c_smbus_access (fd, I2C_SMBUS_WRITE, reg, I2C_SMBUS_BYTE_DATA, &data) ;
}

short read_data(int fd,int addr){
	short high_byte,low_byte,value;
	high_byte = read_reg(fd, addr);
	low_byte = read_reg(fd, addr+1);
	value = (high_byte << 8) | low_byte;
	return value;
}

void GY510_init(int fd){
	write_reg(fd, SMPLRT_DIV, 0x07);	/* Write to sample rate register */
	write_reg(fd, PWR_MGMT_1, 0x01);	/* Write to power management register */
	write_reg(fd, CONFIG, 0);		/* Write to Configuration register */
	write_reg(fd, GYRO_CONFIG, 24);	/* Write to Gyro Configuration register */
	write_reg(fd, INT_ENABLE, 0x01);	/*Write to interrupt enable register */
} 

int main(int argc, const char **argv)
{
    float Acc_x,Acc_y,Acc_z;
	float Gyro_x,Gyro_y,Gyro_z;
	float Ax=0, Ay=0, Az=0;
	float Gx=0, Gy=0, Gz=0;

    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd == -1) {
       perror("Error opening file");   
       exit(1);   
    }
    printf("opened file");
    
    if (ioctl(fd, I2C_SLAVE, Device_Address) == -1){
       perror("Error setting address");   
       exit(1);   
    }

    GY510_init(fd);
    while(1) {
        Acc_x = read_data(fd,ACCEL_XOUT_H);
        Acc_y = read_data(fd,ACCEL_YOUT_H);
        Acc_z = read_data(fd,ACCEL_ZOUT_H);
            
        Gyro_x = read_data(fd,GYRO_XOUT_H);
        Gyro_y = read_data(fd,GYRO_YOUT_H);
        Gyro_z = read_data(fd,GYRO_ZOUT_H);
            
        /* Divide raw value by sensitivity scale factor */
        Ax = Acc_x/16384.0;
        Ay = Acc_y/16384.0;
        Az = Acc_z/16384.0;
            
        Gx = Gyro_x/131;
        Gy = Gyro_y/131;
        Gz = Gyro_z/131;
            
        printf("\n Gx=%.3f °/s\tGy=%.3f °/s\tGz=%.3f °/s\tAx=%.3f g\tAy=%.3f g\tAz=%.3f g\n",Gx,Gy,Gz,Ax,Ay,Az);
        sleep(1);
    }
    return EXIT_SUCCESS;
}