/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

// Port numbers for robot devices, controllers/gamepads, subsystem/mechanism motor controllers, solenoids, and sensors are defined in this file.

/**
 * ROBOT MOTOR CONTROL LAYOUT
 * 
 *  - D = Drivetrain Motors / Motor Controllers
 *  - A = Arm Motors / Motor Controllers
 *  - I = Intake Motors / Motor Controllers
 *  - C = Cascade Motors / Motor Controllers
 *  - FX = TalonFX (Falcon 500) Motor / Motor Controller
 * 
 *       _|_   ___   ___   ___
 *      | 8 | | 7 | | 6 | | 5 |
 *      |___| |___| |___| |___|
 *       A-FX   C     D     D
 *            __________
 *  BACK     |   PDP    |        FRONT >>>>
 *           |__________|
 *                                 ________
 *       _|_   ___   ___   ___    |        |
 *      | 4 | | 3 | | 2 | | 1 |   |roboRIO |
 *      |___| |___| |___| |___|   |________|
 *       A-FX   I     D     D
 */

package com.sentinels.robot.constants;

public interface Ports {

    public interface Devices {
        // Power Distribution Panel (PDP)
        int PDP = 0;

        // Pneumatics Control Module (PCM)
        int PCM = 11;
    }

    public interface Controllers {
        // Controllers
        int DRIVER = 0;
        int OPERATOR = 1;
    }
    
    public interface Arm {
        // Motors
        int ARMLEFTPULLEY = 8;
        int ARMRIGHTPULLEY = 4;
        int ARMCASCADE = 7;
    }

    public interface Intake {
        // Motors
        int INTAKEPIVOT = 3;

        // Solenoids
        int SOLENOIDPUSH = 0;
        int SOLENOIDPULL = 1;
    }

    public interface Drivetrain {
        // Motors
        int FRONTLEFT = 5;
        int BACKLEFT = 6;

        int FRONTRIGHT = 1;
        int BACKRIGHT = 2;
    }
}