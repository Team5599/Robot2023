/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.arm;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.util.RoboRIO;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Code to allow the arm to move.
 * 
 * <p>Arm contains:
 * <p>- 1x Falcon 500 Motor on the LEFT side pulley
 * <p>- 1x Falcon 500 Motor on the RIGHT side pulley
 * <p>- 1x Falcon 500 Motor on the arm for cascade (extend, retract)
 * 
 * @author Ahmed Osman, Karamat Hasan
 */
public class Arm extends SubsystemBase {

  private final WPI_TalonFX armPullL = new WPI_TalonFX(Ports.Arm.ARMLEFTPULLEY);
  private final WPI_TalonFX armPullR = new WPI_TalonFX(Ports.Arm.ARMRIGHTPULLEY);
  private final WPI_TalonFX armCascade = new WPI_TalonFX(Ports.Arm.ARMCASCADE);

  private final WPI_CANCoder encoderL = new WPI_CANCoder(Ports.Arm.ARMLEFTPULLEY);
  private final WPI_CANCoder encoderR = new WPI_CANCoder(Ports.Arm.ARMRIGHTPULLEY);
  private final WPI_CANCoder encoderCascade = new WPI_CANCoder(Ports.Arm.ARMCASCADE);

  private final MotorControllerGroup armPivotMotors = new MotorControllerGroup(armPullL, armPullR);

  public Arm() {
    armPullL.setNeutralMode(NeutralMode.Brake);
    armPullR.setNeutralMode(NeutralMode.Brake);
    armCascade.setNeutralMode(NeutralMode.Brake);

    resetEncoders();
    
    armPullL.setInverted(true);
  }
  
  // motor stall is detected by the output current of a motor
  // since both motors will be rotating together, it is assumed that they will also stall together
  // so they use the same method to check if they are stalling

  //may want to add isStalling as an interrupt to its commands
  /*
  public boolean isStalling() {
    boolean currentL = getMotorCurrents()[0] > 90; // according to the revrobotics neo manual, the stall current is at 105 amps
    boolean currentR = getMotorCurrents()[1] > 90;
    //neo motors are rated to reach a max rpm of 5676 and it has a Kv 473(rpm * Kv = max rpm)
    //we need to find the expected speed with the load of the arm and then check if the motor velocity is much lower
    return currentL && currentR;
  }*/

  public void PivotArm(double speed) {
    armPivotMotors.set(speed);
  }

  public void StopArm() {
    armPivotMotors.stopMotor();
  }

  public void CascadeArm(double speed) {
    //if (isStalling()) { return; }
    armCascade.set(speed);
  }

  public void StopCascade() {
    armCascade.stopMotor();
  }

  public void resetEncoders() {
    encoderL.configFactoryDefault();
    encoderR.configFactoryDefault();
    encoderCascade.configFactoryDefault();

    encoderL.setPosition(0);
    encoderR.setPosition(0);
    encoderCascade.setPosition(0);
  }

  // POSITION METHODS (degrees)

  public double getLeftPosition() {
    return encoderL.getPosition();
  }
  public double getRightPosition() {
    return encoderR.getPosition();
  }
  public double getCascadePosition() {
    return encoderCascade.getPosition();
  }

  // VELOCITY METHODS (RPM) (converted deg/sec to rpm)

  public double getLeftVelocity() {
    return (encoderL.getVelocity() * 6.0);
  }
  public double getRightVelocity() {
    return (encoderR.getVelocity() * 6.0);
  }
  public double getCascadeVelocity() {
    return (encoderCascade.getVelocity() * 6.0);
  }

  // VOLTAGE METHODS (V)

  public double getLeftVoltage() {
    return (armPullL.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (armPullR.get() * RoboRIO.getBatteryVoltage());
  }
  public double getCascadeVoltage() {
    return (armCascade.get() * RoboRIO.getBatteryVoltage());
  }

  // TEMPERATURE METHODS (C)

  public double getLeftTemp() {
    return armPullL.getTemperature();
  }
  public double getRightTemp() {
    return armPullR.getTemperature();
  }
  public double getCascadeTemp() {
    return armCascade.getTemperature();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Left Motor Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Arm/Right Motor Voltage (V)", getRightVoltage());
    SmartDashboard.putNumber("Arm/Cascade Motor Voltage (V)", getCascadeVoltage());

    SmartDashboard.putNumber("Arm/Left Motor Position (Degrees)", getLeftPosition());
    SmartDashboard.putNumber("Arm/Right Motor Position (Degrees)", getRightPosition());
    SmartDashboard.putNumber("Arm/Cascade Motor Position (Degrees)", getCascadePosition());

    SmartDashboard.putNumber("Arm/Left Motor Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Arm/Right Motor Velocity (RPM)", getRightVelocity());
    SmartDashboard.putNumber("Arm/Cascade Motor Velocity (RPM)", getCascadeVelocity());

    SmartDashboard.putNumber("Arm/Debug/Left Motor Temperature (C)", getLeftTemp());
    SmartDashboard.putNumber("Arm/Debug/Right Motor Temperature (C)", getRightTemp());
    SmartDashboard.putNumber("Arm/Debug/Cascade Motor Temperature (C)", getCascadeTemp());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
