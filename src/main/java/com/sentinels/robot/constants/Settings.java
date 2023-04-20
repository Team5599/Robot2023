/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public interface Settings {

    public interface Drivetrain {
        
        double kDriveSpeedCap = 0.8;
        double kTimedDriveVoltage = 2.5;
        double kTimedDriveTime = 3.9; // seconds

        double kGearRatio = 8.45; // 8.45:1 Ratio
        double kDriveBaseWeight = 22.6796; // 50 lbs.
        double kWheelDiameter = 6; // inches
        double kWheelTrackWidth = 22.9; // inches
        double kWheelCircumference = 2 * Math.PI * Units.inchesToMeters(3); // meters
        double kCenterMomentInertia = 5.1349; // kg/m^2
        int kGearboxMotorCount = 2;
        
        DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(Drivetrain.kWheelTrackWidth));
    }

    public static interface Arm {

        double kArmSpeedCap = 0.7;
        double kPivotTimedSpeed = 0.4;

        double kArmLength = 47; // inches
        double kCascadeLength = 40; // inches
        double kCascadeMaxExtensionLength = 21;
        double kPivotToPerimeter = 30; // distance from pivot to the bumpers/ the robot perimeter
        double kMaxCascadeLength = 68; // from the pivot to the end of the arm once full extended in inches

        double kPivotStartingAngle = 90 - 27; // may need extra data 
        double kArmPivotGearRatio = 27; // 27:1 Ratio

        double kTOPpivot = 2;
        double kMEDIUMpivot = 33;

        public enum Level { // enum to be used in certain commands

            TOP(0,0),
            MEDIUM(33,0),
            LOW(0,0);

            // for now, these are for the cube
            public double pivotAngle;
            public double cascadeLength;

            Level(double pivotAngle, double cascadeLength) {
                this.cascadeLength = cascadeLength;
                this.pivotAngle = pivotAngle;
            }
        }
    }
}