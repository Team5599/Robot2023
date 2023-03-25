// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.constants.Settings.Drivetrain.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonTimedDrive extends CommandBase {

  private Drivetrain drivetrain;
  private boolean reverseEnabled;

  public AutonTimedDrive(Drivetrain drivetrain, boolean reverseEnabled) {
    this.drivetrain = drivetrain;
    this.reverseEnabled = reverseEnabled;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (reverseEnabled) {
      drivetrain.voltageDrive(-Autonomous.kTimedDriveSpeed, -Autonomous.kTimedDriveSpeed);
    } else {
      drivetrain.voltageDrive(Autonomous.kTimedDriveSpeed, Autonomous.kTimedDriveSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
