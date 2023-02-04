/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.odometry;

import edu.wpi.first.wpilibj.ADIS16470_IMU;

public class IMU {

    ADIS16470_IMU imu = new ADIS16470_IMU();
    
    public IMU() {
        if (imu.isConnected() != true) {
            // handle connection issue?
        }
    }
}
