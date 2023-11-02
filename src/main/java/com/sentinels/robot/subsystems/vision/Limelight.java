/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.PhotonCamera;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry viewStream = table.getEntry("stream");
    NetworkTableEntry getled = table.getEntry("ledMode");

    // private PhotonCamera

    public Limelight() {

    }

    @Override
    public void periodic() {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("Limelight/tX", x);
        SmartDashboard.putNumber("Limelight/tY", y);
        SmartDashboard.putNumber("Limelight/tA", area);

        blink();
        sidestream();
    }
    
    // these will return -191 if there is no game piece present
    public double getTx() {//gets xoffset angle
        return table.getEntry("tx").getDouble(200);// 200 since it is not an angle that can be returned
    }

    public double getTa() {
        return table.getEntry("ta").getDouble(-200);
    }

    public void blink() {
        table.getEntry("ledMode").setNumber(1); // Set value to 2 for blink
    }
    
    public void sidestream() {
        table.getEntry("stream").setNumber(0);
    }

    public double ParllaxDistance(double angle1, double angle2) {
        double distance = Math.sin(angle1) / Math.sin(angle2 - angle1);
        return distance;
    }
}