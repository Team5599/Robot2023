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
        int ARMLEFT = 0;
        int ARMRIGHT = 0;
        int ARMPULLEY = 0;
    }

    public interface ArmIntake {
        // Pneumatics
        int PNEUMATICS = 0; // temp var name until pneumatics are known
    }

    public interface Drivetrain {
        // Motors
        int FRONTLEFT = 0;
        int BACKLEFT = 0;

        int FRONTRIGHT = 0;
        int BACKRIGHT = 0;
    }
}