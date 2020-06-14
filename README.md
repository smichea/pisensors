# pisensors
Raspberry Pi sensor management in java
is a java program that *plan* to aggregate data from different sensors and send it to some client via BLE, wifi or USB

This software currently handle
* GY-511 gyroscope

## installation
on a raspbian, install java 8 (we need to wait for Pi4J v2 to upgrade)
```
sudo apt get openjdk-8-jdk
```

On a development pc or directly in the Pi, clone this repo and compile using maven.
then copy the pisensors.jar and /libs directory to some place in your Pi then execute
```
java -jar pisensors.jar
```
