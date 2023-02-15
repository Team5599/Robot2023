/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.odometry;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;

public class IMU {

    private final ADIS16470_IMU imu = new ADIS16470_IMU();
    
    public IMU() {
        if (imu.isConnected() != true) {
            // handle connection issue?
        }
    }

    public double getAccelX(){
        return imu.getAccelX(); //right is positive to the imu
    }
    public double getAccelY(){
        return imu.getAccelY(); //forward is positive 
    }
    public double getAccelZ(){
        return imu.getAccelZ(); //up is positive
    }
    public double getPitch(){
        return imu.getYFilteredAccelAngle();// will get the pitch of the robots angle with acceleration filtered out
    }
}
   