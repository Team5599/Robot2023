/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.constants;

// Port numbers for controllers, subsystems/mechanisms, and sensors are defined in this file.

public interface Ports {

    public interface Controllers {
        int DRIVER = 0;
        int OPERATOR = 1;
    }
    
    public interface Arm {
        int MOTOR_1 = 0;
        int MOTOR_2 = 0;
        int MOTOR_3 = 0;
    }

    public interface ArmIntake {
        int PNEUMATICS = 0; // temp var name until pneumatics are known
    }

    public interface Drivetrain {
        int FRONTLEFT = 0;
        int BACKLEFT = 0;

        int FRONTRIGHT = 0;
        int BACKRIGHT = 0;
    }
}