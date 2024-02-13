
#ifndef IMU_H
#define IMU_H


class IMU
{
public:

    float accelXoffset, accelYoffset,accelZoffset;
    float gyroXoffset, gyroYoffset,gyroZoffset;
    float yaw,pitch,roll;

    void calibrate();
    void settings();
    void mag();
    void readData();

};


#endif