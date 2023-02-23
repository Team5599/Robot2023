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
    }

}