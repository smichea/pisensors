# pisensors
Raspberry Pi sensor management in java
is a java program that *plan* to aggregate data from different sensors and send it to some client via BLE, wifi or USB

This software currently handle
* GY-511 gyroscope

## installation
on a raspbian, [install java 8](https://linuxize.com/post/install-java-on-debian-10/) (we need to wait for Pi4J v2 to upgrade) 

```
sudo apt update
sudo apt install apt-transport-https ca-certificates wget dirmngr gnupg software-properties-common
wget -qO - https://adoptopenjdk.jfrog.io/adoptopenjdk/api/gpg/key/public | sudo apt-key add -
sudo add-apt-repository --yes https://adoptopenjdk.jfrog.io/adoptopenjdk/deb/
sudo apt update
sudo apt install adoptopenjdk-8-hotspot
```

On a development pc or directly in the Pi, clone this repo and compile using maven.
then copy the pisensors.jar and /libs directory to some place in your Pi then execute
```
java -jar pisensors.jar
```

it will launch a websocket server on port 6340

a javascript client that you can open in a browser is [here](https://github.com/smichea/pisensors/tree/master/src/main/js)

## 
On 'data_fusion' directory, execute following commands to compile C++ codes:

```
g++ -pthread  -o ReadRawData ReadRawData.cpp ./MPU6050_library/MPU6050.cpp ./i2c_library/smbus.c
```

```
g++ -pthread  -o BiasCalculator BiasCalculator.cpp ./MPU6050_library/MPU6050.cpp ./i2c_library/smbus.c
```

```
g++ -pthread  -o ReadAngles ReadAngles.cpp ./MPU6050_library/MPU6050.cpp ./i2c_library/smbus.c
```



