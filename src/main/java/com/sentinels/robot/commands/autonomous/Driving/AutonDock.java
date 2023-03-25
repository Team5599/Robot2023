// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving;

import com.sentinels.robot.subsystems.drive.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDock extends CommandBase {

  private final Drivetrain drivetrain;
  private PIDController balanceController;

  /** Creates a new AutonDock. */
  public AutonDock(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    balanceController = new PIDController(.1, 0, 0.1);
    balanceController.setSetpoint(0);
    addRequirements(drivetrain);// might need the IMU in here
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    balanceController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double input = balanceController.calculate(drivetrain.getPitch());
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
    if(balanceController.atSetpoint()){
      return true;
    }
    return false;
  }
}
