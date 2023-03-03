/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.subsystems.intake;

import com.sentinels.robot.constants.Ports;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmIntake extends SubsystemBase {

  //private final CANSparkMax motorPivot = new CANSparkMax(Ports.ArmIntake.INTAKEPIVOT, null);
  private final WPI_TalonFX armPivot = new WPI_TalonFX(Ports.ArmIntake.INTAKEPIVOT);
  private final WPI_CANCoder encoderPivot = new WPI_CANCoder(Ports.ArmIntake.INTAKEPIVOT);

  public ArmIntake() {
  }

  // may need a pid controller?
  public void setSpeed(double speed){
    armPivot.set(speed);
  }

  public void getSpeed(){
    armPivot.get();
  }
  public double getVelocity(){
    return armPivot.getSelectedSensorVelocity();
  }
  

  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  @Override
  public void periodic() {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
