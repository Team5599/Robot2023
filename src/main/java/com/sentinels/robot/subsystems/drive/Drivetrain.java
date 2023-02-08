/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.drive;

import com.sentinels.robot.constants.Motors;
import com.sentinels.robot.constants.Ports;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Code to allow the robot to move.
 * 
 * <p>Tank Drive contains:
 * <p>- 2x NEO Motors on the LEFT side (front and back)
 * <p>- 2x NEO Motors on the RIGHT side (front and back)
 * 
 * @author Ahmed Osman, Karamat Hasan
 */
public class Drivetrain extends SubsystemBase {

  // LEFT side motors
  CANSparkMax motorFL = new CANSparkMax(Ports.Drivetrain.FRONTLEFT, MotorType.kBrushless);
  CANSparkMax motorBL = new CANSparkMax(Ports.Drivetrain.BACKLEFT, MotorType.kBrushless);
  // RIGHT side motors
  CANSparkMax motorFR = new CANSparkMax(Ports.Drivetrain.FRONTRIGHT, MotorType.kBrushless);
  CANSparkMax motorBR = new CANSparkMax(Ports.Drivetrain.BACKRIGHT, MotorType.kBrushless);

  MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL);
  MotorControllerGroup rightMotors = new MotorControllerGroup(motorFR, motorBR);
  DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);


  public Drivetrain() {
    // Invert the one of the sides so that they rotate synonymously in one direction
    leftMotors.setInverted(true);
  }

  /**
   * Drive the robot!
   * @param leftSpeed - The speed at which the left side motors should be.
   * @param rightSpeed - The speed at which the right side motors should be.
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    drivetrain.tankDrive(leftSpeed, rightSpeed);
  }

  // Stop the motors from moving
  public void driveStop() {
    drivetrain.stopMotor();
  }

  /**
   * Manually change the speed of both MotorControllerGroups {@code leftMotors} and {@code rightMotors}.
   * @param leftSpeed - The speed at which the left side motors should be.
   * @param rightSpeed - The speed at which the right side motors should be.
   */
  public void setSpeed(double leftSpeed, double rightSpeed) {
    leftMotors.set(leftSpeed);
    rightMotors.set(rightSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
