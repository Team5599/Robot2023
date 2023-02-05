/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.commands.drivetrain;

import com.sentinels.robot.subsystems.drive.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Autonomous-Exclusive Command
 */
public class DrivetrainTurn extends CommandBase {
    
    private final Drivetrain m_Drivetrain;
    private final int direction;
    // initial values for left and right speeds
    private final double leftSpeed;
    private final double rightSpeed;

    /**
     * Turns the robot a specified amount and direction.
     * <p> One of the speed arguments must be a non-positive value.
     * 
     * @param drivetrain - Drivetrain subsystem argument.
     * @param leftSpeed - Speed for the left motors.
     * @param rightSpeed - Speed for the right motors.
     * @param direction - Direction in which to turn in. {@code 0} for LEFT, {@code 1} for RIGHT. 
     * @author Ahmed Osman
     */
    public DrivetrainTurn(Drivetrain drivetrain, double leftSpeed, double rightSpeed, int direction) {

        this.m_Drivetrain = drivetrain;
        this.leftSpeed = leftSpeed;
        this.rightSpeed = rightSpeed;
        this.direction = direction;

        addRequirements(drivetrain);
    }

    public void tankDriveTurn(double leftSpeed, double rightSpeed) {
        m_Drivetrain.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void execute() {
        tankDriveTurn(leftSpeed, rightSpeed);
    }
}
