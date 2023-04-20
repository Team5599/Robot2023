// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

//DOES NOT WORK
public class PIDdrive extends CommandBase {

  private final Drivetrain drivetrain;
  private final PIDController controller;
  private final double setpoint;

  public PIDdrive(Drivetrain drivetrain, double setpoint) {
    this.drivetrain = drivetrain;
    this.setpoint = setpoint;

    controller = new PIDController(2, 0, 1);
    controller.setSetpoint(this.setpoint);

    System.out.println(controller.getPeriod());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = MathUtil.clamp(controller.calculate(drivetrain.getLeftPosition()), -0.7, 0.7);
    drivetrain.tankDrive(input, input);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(controller.atSetpoint()){
      return true;
    }
    return false;
  }
}
