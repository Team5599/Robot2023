/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.intake;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.util.RoboRIO;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Code to allow the intake to compress, release, and pivot.
 * 
 * Intake contains:
 * </p>- 1x NEO 550 motor w/ encoder for pivot
 * </p>- 1x Double solenoid for pneumatic piston for push and pull
 */
public class Intake extends SubsystemBase {

  private final CANSparkMax motorPivot = new CANSparkMax(Ports.Intake.INTAKEPIVOT, MotorType.kBrushless);

  private final RelativeEncoder encoderPivot = motorPivot.getEncoder();

  private final DoubleSolenoid solenoid = new DoubleSolenoid(
    PneumaticsModuleType.CTREPCM, Ports.Intake.SOLENOIDPUSH, Ports.Intake.SOLENOIDPULL
  );
  private final Compressor compressor = new Compressor(PneumaticsModuleType.CTREPCM);

  public Intake() {
    // Reset motor and encoder
    motorPivot.restoreFactoryDefaults();
    encoderPivot.setPosition(0);

    solenoid.set(DoubleSolenoid.Value.kOff);
  }

  public void intakeConstrict() {
    solenoid.set(Value.kForward);
  }
  
  public void intakeRetract() {
    solenoid.set(Value.kReverse);
  }

  // ENCODER + MOTOR INFO METHODS

  public double getPivotVoltage() {
    return (motorPivot.get() * RoboRIO.getBatteryVoltage());
  }
  public double getPivotPosition() {
    return encoderPivot.getPosition();
  }
  public double getPivotVelocity() {
    return encoderPivot.getVelocity();
  }

  public String getSolenoidState() {
    return solenoid.get().toString();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("Intake/Solenoid Piston State", getSolenoidState());
    SmartDashboard.putNumber("Intake/Pivot Motor Position", getPivotPosition());
    SmartDashboard.putNumber("Intake/Pivot Motor Velocity (RPM)", getPivotVelocity());
    SmartDashboard.putNumber("Intake/Pivot Motor Voltage (V)", getPivotVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
