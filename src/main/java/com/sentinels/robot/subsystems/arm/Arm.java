/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.arm;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.util.RoboRIO;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Code to allow the arm to move.
 * 
 * <p>Arm contains:
 * <p>- 1x NEO Motor on the LEFT side
 * <p>- 1x NEO Motor on the RIGHT side
 * <p>- 1x NEO Motor in the MIDDLE for the pulley
 * 
 * @author Ahmed Osman, Karamat Hasan
 */
public class Arm extends SubsystemBase {

  private final CANSparkMax ArmL = new CANSparkMax(Ports.Arm.ARMLEFT, MotorType.kBrushless);
  private final CANSparkMax ArmR = new CANSparkMax(Ports.Arm.ARMRIGHT, MotorType.kBrushless);
  private final CANSparkMax ArmPulley = new CANSparkMax(Ports.Arm.ARMPULLEY, MotorType.kBrushless);

  private final MotorControllerGroup ArmMotors = new MotorControllerGroup(ArmL, ArmR);

  private final RelativeEncoder encoderL = ArmL.getEncoder();
  private final RelativeEncoder encoderR = ArmL.getEncoder();
  private final RelativeEncoder encoderPulley = ArmL.getEncoder();

  public Arm() {
    ArmL.setInverted(true);
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
    ArmPulley.set(velocity);
  }
  public void ElevatorStop() {
    ArmPulley.set(0);
  }

  // POSITION METHODS

  public double getLeftPosition() {
    return encoderL.getPosition();
  }
  public double getRightPosition() {
    return encoderR.getPosition();
  }
  public double getPulleyPosition() {
    return encoderPulley.getPosition();
  }

  // VELOCITY METHODS (RPM)

  public double getLeftVelocity() {
    return encoderL.getVelocity();
  }
  public double getRightVelocity() {
    return encoderR.getVelocity();
  }
  public double getPulleyVelocity() {
    return encoderPulley.getVelocity();
  }

  // VOLTAGE METHODS (V)

  public double getLeftVoltage() {
    return (ArmL.get() * RoboRIO.getBatteryVoltage());
  }
  public double getRightVoltage() {
    return (ArmR.get() * RoboRIO.getBatteryVoltage());
  }
  public double getPulleyVoltage() {
    return (ArmPulley.get() * RoboRIO.getBatteryVoltage());
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
