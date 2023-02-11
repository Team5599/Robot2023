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
        int DRIVER = 1;
        int OPERATOR = 2;
    }
    
    public interface Arm {
        // Motors
        int ARMLEFT = 4;
        int ARMRIGHT = 5;
        int ARMPULLEY = 6;
    }

    public interface ArmIntake {
        // Pneumatics
        int PNEUMATICS = 0; // temp var name until pneumatics are known
    }

    public interface Drivetrain {
        // Motors
        int FRONTLEFT = 0;
        int BACKLEFT = 1;

        int FRONTRIGHT = 2;
        int BACKRIGHT = 3;
    }
}