/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.constants;

public interface Settings {

    public interface Drivetrain {
        double DRIVESPEEDCAP = 0.8;

        double GEAR_RATIO = 8.45; // 8.45:1 Ratio
        double DRIVEBASE_KG = 22.6796; // 50 lbs.
        double WHEEL_DIAMETER = 6; // inches
        double WHEEL_TRACK_WIDTH = 22.9; // inches
        double CENTER_MOMENT_INERTIA = 5.1349; // kg/m^2
        int GEARBOX_NUM_MOTORS = 2;
    }

}