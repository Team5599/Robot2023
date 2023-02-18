/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.drive;

import com.sentinels.robot.constants.Motors;
import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.util.RoboRIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final CANSparkMax motorFL = new CANSparkMax(Ports.Drivetrain.FRONTLEFT, MotorType.kBrushless);
  private final CANSparkMax motorBL = new CANSparkMax(Ports.Drivetrain.BACKLEFT, MotorType.kBrushless);
  
  // RIGHT side motors
  private final CANSparkMax motorFR = new CANSparkMax(Ports.Drivetrain.FRONTRIGHT, MotorType.kBrushless);
  private final CANSparkMax motorBR = new CANSparkMax(Ports.Drivetrain.BACKRIGHT, MotorType.kBrushless);

  private final MotorControllerGroup leftMotors = new MotorControllerGroup(motorFL, motorBL);
  private final MotorControllerGroup rightMotors = new MotorControllerGroup(motorFR, motorBR);

  private final DifferentialDrive drivetrain = new DifferentialDrive(leftMotors, rightMotors);

  // Built-in NEO hall sensor encoders
  private final RelativeEncoder encoderFL = motorFL.getEncoder();
  private final RelativeEncoder encoderBL = motorBL.getEncoder();
  private final RelativeEncoder encoderFR = motorFR.getEncoder();
  private final RelativeEncoder encoderBR = motorBR.getEncoder();


  public Drivetrain() {
    // Resetting motor settings to default factory settings
    motorFL.restoreFactoryDefaults();
    motorBL.restoreFactoryDefaults();
    motorFR.restoreFactoryDefaults();
    motorBR.restoreFactoryDefaults();

    // Zeroing encoder positions
    encoderFL.setPosition(0);
    encoderBL.setPosition(0);
    encoderFR.setPosition(0);
    encoderBR.setPosition(0);

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

  //PID
  

  // POSITION FUNCTIONS

  public double getLeftPosition() {
    return (encoderFL.getPosition() + encoderBL.getPosition() / 2.0);
  }
  public double getRightPosition() {
    return (encoderFR.getPosition() + encoderBR.getPosition() / 2.0);
  }

  // VELOCITY FUNCTIONS

  public double getLeftVelocity() {
    return (encoderFL.getVelocity() + encoderBL.getVelocity() / 2.0);
  }
  public double getRightVelocity() {
    System.out.println(encoderFR.getVelocity());
    System.out.println(encoderBR.getVelocity());
    return -(encoderFR.getVelocity() + encoderBR.getVelocity() / 2.0);
  }

  public double getRightFrontVel() {
    return -(encoderFR.getVelocity());
  }
  public double getRightBackVel() {
    return -(encoderBR.getVelocity());
  }

  // VOLTAGE FUNCTIONS

  public double getLeftVoltage() {
    return (leftMotors.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (rightMotors.get() * RoboRIO.getBatteryVoltage());
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drivetrain Left Motors Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Drivetrain Right Motors Voltage (V)", getRightVoltage());

    SmartDashboard.putNumber("Drivetrain Left Motors Position (Rotations)", getLeftPosition());
    SmartDashboard.putNumber("Drivetrain Right Motors Position (Rotations)", getRightPosition());

    SmartDashboard.putNumber("Drivetrain Left Motors Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Drivetrain Right Motors Velocity (RPM)", getRightVelocity());
    SmartDashboard.putNumber("Drivetrain Right Front Motor Vel", getRightFrontVel());
    SmartDashboard.putNumber("Drivetrain Right Back Motor Vel", getRightBackVel());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
