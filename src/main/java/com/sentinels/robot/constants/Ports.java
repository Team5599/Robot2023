/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

// Port numbers for controllers/gamepads, subsystem/mechanism motor controllers, and sensors are defined in this file.

/**
 * ROBOT MOTOR CONTROL LAYOUT
 * 
 *  - D = Drivetrain Motors / Motor Controllers
 *  - A = Arm Motors / Motor Controllers
 *  - I = Intake Motors / Motor Controllers
 *  - P = Pulley Motors / Motor Controllers
 * 
 *       ___   ___   ___   ___
 *      | 1 | | 2 | | 3 | | 4 |
 *      |___| |___| |___| |___|
 *        D     D     A     I
 *            __________
 *  BACK     |   PDP    |        FRONT >>>>
 *           |__________|
 *                                 ________
 *       ___   ___   ___   ___    |        |
 *      | 5 | | 6 | | 7 | | 8 |   |roboRIO |
 *      |___| |___| |___| |___|   |________|
 *        D     D     A     P
 */

package com.sentinels.robot.constants;

public interface Ports {

    public interface Controllers {
        // Controllers
        int DRIVER = 0;
        int OPERATOR = 1;
    }
    
    public interface Arm {
        // Motors
        int ARMLEFTPULLEY = 7;
        int ARMRIGHTPULLEY = 3;
        int ARMCASCADE = 8;
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