/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.arm;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.util.RoboRIO;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
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

  //private final TalonFX armTest = new TalonFX(2);
  private final WPI_TalonFX ArmPullL = new WPI_TalonFX(Ports.Arm.ARMLEFTPULLEY);
  private final WPI_TalonFX ArmPullR = new WPI_TalonFX(Ports.Arm.ARMRIGHTPULLEY);
  private final WPI_TalonFX ArmCascade = new WPI_TalonFX(Ports.Arm.ARMCASCADE);


  private final WPI_CANCoder encoderL = new WPI_CANCoder(Ports.Arm.ARMLEFTPULLEY);
  private final WPI_CANCoder encoderR = new WPI_CANCoder(Ports.Arm.ARMRIGHTPULLEY);
  private final WPI_CANCoder encoderCascade = new WPI_CANCoder(Ports.Arm.ARMCASCADE);



  private final MotorControllerGroup ArmMotors = new MotorControllerGroup(ArmPullL,ArmPullR);
  public Arm() {
    ArmPullL.setInverted(true);
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

  public void setArmSpeed(double speed) {
    ArmMotors.set(speed);
  }
  public void ExtendArm(double armExtendSpeed) {
    //if (isStalling()) { return; }
    ArmMotors.set(armExtendSpeed);
  }
  public void RetractArm(double armRetractSpeed) {
    //if (isStalling()) { return; }
    ArmMotors.set(armRetractSpeed);
  }
  public void StopArm() {
    ArmMotors.set(0);
  }

  // the pulley may also need its own stall checks
  public void setPulley(double velocity) {
    ArmCascade.set(velocity);
  }
  public void ElevatorStop() {
    ArmCascade.set(0);
  }

  // POSITION METHODS

  public double getLeftPosition() {
    return encoderL.getPosition();
    // return encoderL.getPosition();
  }
  public double getRightPosition() {
    return encoderR.getPosition();
  }
  public double getPulleyPosition() {
    return encoderCascade.getPosition();
  }

  // VELOCITY METHODS (RPM)

  public double getLeftVelocity() {
    return encoderL.getVelocity();
  }
  public double getRightVelocity() {
    return encoderR.getVelocity();
  }
  public double getPulleyVelocity() {
    return encoderCascade.getVelocity();
  }

  // VOLTAGE METHODS (V)

  public double getLeftVoltage() {
    return (ArmPullL.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (ArmPullR.get() * RoboRIO.getBatteryVoltage());
  }
  public double getPulleyVoltage() {
    return (ArmCascade.get() * RoboRIO.getBatteryVoltage());
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm/Left Motor Voltage (V)", getLeftVoltage());
    SmartDashboard.putNumber("Arm/Right Motor Voltage (V)", getRightVoltage());
    SmartDashboard.putNumber("Arm/Pulley Motor Voltage (V)", getPulleyVoltage());

    SmartDashboard.putNumber("Arm/Left Motor Position (Rotations)", getLeftPosition());
    SmartDashboard.putNumber("Arm/Right Motor Position (Rotations)", getRightPosition());
    SmartDashboard.putNumber("Arm/Pulley Motor Position (Rotations)", getPulleyPosition());

    SmartDashboard.putNumber("Arm/Left Motor Velocity (RPM)", getLeftVelocity());
    SmartDashboard.putNumber("Arm/Right Motor Velocity (RPM)", getRightVelocity());
    SmartDashboard.putNumber("Arm/Pulley Motor Velocity (RPM)", getRightVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
