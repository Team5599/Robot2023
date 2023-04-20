// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.drivetrain;

import com.sentinels.robot.subsystems.drive.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DrivetrainVoltageDrive extends CommandBase {

  private final Drivetrain drivetrain;

  public DrivetrainVoltageDrive(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    drivetrain.voltageDrive(8, 10);
  }

  @Override
  public void end(boolean interrupted) {}
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
