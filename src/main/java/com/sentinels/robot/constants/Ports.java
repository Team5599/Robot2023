/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

// Port numbers for controllers, subsystems/mechanisms, and sensors are defined in this file.

package com.sentinels.robot.constants;

public interface Ports {

    public interface Controllers {
        // Controllers
        int DRIVER = 0;
        int OPERATOR = 1;
    }
    
    public interface Arm {
        // Motors
        int ARMLEFT = 7;
        int ARMRIGHT = 3;
        int ARMPULLEY = 8;
    }

    public interface ArmIntake {
        // Pneumatics
        int PNEUMATICS = 10; // temp var name until pneumatics are known
        // Motors
        int INTAKEPIVOT = 4;
    }

    public interface Drivetrain {
        // Motors
        int FRONTLEFT = 5;
        int BACKLEFT = 6;

        int FRONTRIGHT = 1;
        int BACKRIGHT = 2;
    }
}