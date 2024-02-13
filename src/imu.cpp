#include "imu.h"
#include <SparkFunLSM6DS3.h>
#include "QMC6310_Unified.h"
#include <SensorFusion.h>

#define OFFSET_CALCULATION_TIME 3000 // offset calculation time (ms)

QMC6310_Unified mag_sensor = QMC6310_Unified(12345, false);

SF fusion;
LSM6DS3 imu = LSM6DS3(I2C_MODE, 0x6A);

//Calibrate it so that it takes the place it was at when it first started as a starting point
void IMU::calibrate() {
    int samples;
    int start = millis();
    while (millis() < start + OFFSET_CALCULATION_TIME)
    {
        gyroXoffset +=imu.readFloatGyroX();
        gyroYoffset += imu.readFloatGyroY();
        gyroZoffset +=imu.readFloatGyroZ();

        accelXoffset +=imu.readFloatAccelX();
        accelYoffset += imu.readFloatAccelY();
        accelZoffset += imu.readFloatAccelZ();
        samples ++;
    }

    gyroXoffset = gyroXoffset/samples;
    gyroYoffset = gyroYoffset/samples;
    gyroZoffset = gyroZoffset/samples;

    accelXoffset = accelXoffset/samples;
    accelYoffset = accelYoffset/samples;
    accelZoffset = (accelZoffset/samples) - 1;  
}

//Set imu initial values
void IMU::settings()
{
  imu.settings.gyroEnabled = 1;  //Can be 0 or 1
  imu.settings.gyroRange = 2000;   //Max deg/s.  Can be: 125, 245, 500, 1000, 2000
  imu.settings.gyroSampleRate =52;   //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666
  imu.settings.gyroBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  imu.settings.gyroFifoEnabled = 1;  //Set to include gyro in FIFO
  imu.settings.gyroFifoDecimation = 1;  //set 1 for on /1

  imu.settings.accelEnabled = 1;
  imu.settings.accelRange = 16;      //Max G force readable.  Can be: 2, 4, 8, 16
  imu.settings.accelSampleRate = 52;  //Hz.  Can be: 13, 26, 52, 104, 208, 416, 833, 1666, 3332, 6664, 13330
  imu.settings.accelBandWidth = 200;  //Hz.  Can be: 50, 100, 200, 400;
  imu.settings.accelFifoEnabled = 1;  //Set to include accelerometer in the FIFO
  imu.settings.accelFifoDecimation = 1;  //set 1 for on /1

  imu.begin();
}

void IMU:: mag()
{
  mag_sensor.setFieldRange(QMC6310_FLD_RNG_2);
  // set the output data rate (ODR) to 50Hz 
  mag_sensor.setDataRate(QMC6310_DATA_RATE_50);
  // set the over-sampling rate (OSR) to 8x 
  mag_sensor.setOSR(QMC6310_OSR_8);
  mag_sensor.begin();
}

void IMU:: readData()
{
    // Reading data from sensors
    float ax = imu.readFloatAccelX() - accelXoffset;
    float ay = imu.readFloatAccelY() - accelYoffset;
    float az = imu.readFloatAccelZ() - accelZoffset;

    float gx = (imu.readFloatGyroX() -gyroXoffset) * DEG_TO_RAD ;
    float gy = (imu.readFloatGyroY()-gyroYoffset) * DEG_TO_RAD;
    float gz = (imu.readFloatGyroZ()- gyroZoffset) * DEG_TO_RAD;

        
    sensors_event_t event; 
    
    mag_sensor.getEvent(&event);
    // Calculate yaw, pitch, roll values
    float  deltat;
    extern IMU mu;         // used to define the object once. If you want to more information https://stackoverflow.com/questions/10422034/when-to-use-extern-in-c

    deltat = fusion.deltatUpdate(); 
    fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, event.magnetic.x, event.magnetic.y, event.magnetic.z, deltat);  

    mu.pitch = fusion.getPitch();
    mu.roll = fusion.getRoll();    
    mu.yaw = fusion.getYaw();
}
