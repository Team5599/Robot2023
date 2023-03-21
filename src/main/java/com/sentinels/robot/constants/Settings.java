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

        double kGearRatio = 8.45; // 8.45:1 Ratio
        double kDriveBaseWeight = 22.6796; // 50 lbs.
        double kWheelDiameter = 6; // inches
        double kWheelTrackWidth = 22.9; // inches
        double kCenterMomentInertia = 5.1349; // kg/m^2
        int kGearboxMotorCount = 2;
        
        DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(Units.inchesToMeters(Drivetrain.kWheelTrackWidth));

        double kWheelCircumference = 2 * Math.PI * Units.inchesToMeters(3); // meters
    }

    public static interface Arm {
        double kArmSpeedCap = 0.7;

        double kArmLength = 47; // in inches
        double kCascadeLength = 40;// in
        double kCascadeMaxExtensionLength = 21;
        double kPivotToPerimeter = 30;// distance from pivot to the bumpers/ the robot perimeter
        double kMaxCascadeLength = 68;// from the pivot to the end of the arm once full extended in inches

        //double kPivotStartingAngle = 
        double kPivotMaxAngle = 90 - Math.acos(28/38);
        double kPivotGearRatio = 27;// versal planetary gear ratio
        
                
        public enum level {// enum to be used in certain commands
            TOP,
            MEDIUM,
            LOW
        }  
    }

}