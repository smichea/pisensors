# pisensors
Raspberry Pi sensor management in java
is a java program that *plan* to aggregate data from different sensors and send it to some client via BLE, wifi or USB

This software currently handle
* GY-511 gyroscope

## harware installation

![image](https://user-images.githubusercontent.com/16659140/95651679-9b751c80-0b1e-11eb-9ac0-be16d9dfb82e.png)

## software installation

### Raspberry Os
on a raspbian, [install java 8](https://linuxize.com/post/install-java-on-debian-10/) (we need to wait for Pi4J v2 to upgrade) 

```
sudo apt update
sudo apt install apt-transport-https ca-certificates wget dirmngr gnupg software-properties-common
wget -qO - https://adoptopenjdk.jfrog.io/adoptopenjdk/api/gpg/key/public | sudo apt-key add -
sudo add-apt-repository --yes https://adoptopenjdk.jfrog.io/adoptopenjdk/deb/
sudo apt update
sudo apt install adoptopenjdk-8-hotspot
```

### Ubuntu 20
Download [Ubuntu 20 for raspberry pi 4 64bits](https://ubuntu.com/download/raspberry-pi)
and install it on an SD card with [rasperby imager](https://www.raspberrypi.org/downloads/) or other tool like [etcher](https://www.balena.io/etcher/).

Setup the [wifi connection](https://medium.com/@huobur/how-to-setup-wifi-on-raspberry-pi-4-with-ubuntu-20-04-lts-64-bit-arm-server-ceb02303e49b).

Upgrade the system
```
sudo apt update && sudo apt -f install && sudo apt full-upgrade
```

Install the jdk8
```
sudo apt install openjdk-8-jdk-headless
```

Install maven
```
sudo apt install maven
```

## Build the project

Go to `/home/` and clone this repo
```
cd /home/
sudo git clone https://github.com/smichea/pisensors.git
```
Alternatively you can do the compilation on a PC then copy the pisensors.jar and /libs directory to some place in your Pi.


## Run project
got to `/home/pisensors/target` or to the place you copied `pisensors.jar` then execute it
```
java -jar pisensors.jar
```

it will launch a websocket server on port 6340

a javascript client that you can open in a browser is [here](https://github.com/smichea/pisensors/tree/master/src/main/js)



## C++ code that served as a model 

### Install g++

on ubuntu
```
sudo apt install build-essential
````

### compile test programs
On `/home/pisensors/src/main/c/data_fusion/' directory, execute following commands to compile C++ codes:

```
g++ -pthread  -o ReadRawData ReadRawData.cpp ./MPU6050_library/MPU6050.cpp ./i2c_library/smbus.c
```

```
g++ -pthread  -o BiasCalculator BiasCalculator.cpp ./MPU6050_library/MPU6050.cpp ./i2c_library/smbus.c
```

```
g++ -pthread  -o ReadAngles ReadAngles.cpp ./MPU6050_library/MPU6050.cpp ./i2c_library/smbus.c
```



