/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.odometry;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IMU extends SubsystemBase {

    public static final ADIS16470_IMU imu = new ADIS16470_IMU();

    private double newAngle;
    private double oldAngle;
    
    public IMU() {
        if (imu.isConnected() != true) {
            System.out.println("CRITICAL: IMU not connected!");
        }
    }

    public double getAccelX() {
        return imu.getAccelX(); //right is positive to the imu
    }
    public double getAccelY() {
        return imu.getAccelY(); //forward is positive 
    }
    public double getAccelZ() {
        return imu.getAccelZ(); //up is positive
    }
    public double getPitch() {
        return imu.getYFilteredAccelAngle();// will get the pitch of the robots angle with acceleration filtered out
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Change in IMU pitch: ", getDeltaAngle(getPitch()));
    }

    public double getDeltaAngle(double angle) {
        newAngle = angle;
        oldAngle = newAngle;

        return (newAngle - oldAngle);
    }
}
   