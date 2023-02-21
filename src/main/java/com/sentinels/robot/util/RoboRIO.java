/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.util;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;

/**
 * RoboRIO control/getter methods & variables.
 * 
 * @author Ahmed Osman
 */
public class RoboRIO {
    
    public static double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    public static double getInputVoltage() {
        return RobotController.getInputVoltage();
    }

    public static boolean isBrownedOut() {
        return RobotController.isBrownedOut();
    }
    
    public static CANStatus getCANStatus() {
        return RobotController.getCANStatus();
    }

}
