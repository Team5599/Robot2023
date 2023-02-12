// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.drivetrain;

import com.sentinels.robot.subsystems.drive.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DrivetrainDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain m_Drivetrain;
  private final CommandXboxController driver;

  double driverLeftY;
  double driverRightY;
  
  public DrivetrainDrive(Drivetrain drivetrain, CommandXboxController controller) {
    this.m_Drivetrain = drivetrain;
    this.driver = controller;

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    driverLeftY = 0;
    driverRightY = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driverLeftY = driver.getLeftY();
    driverRightY = driver.getRightY();

    // Make both input values negative due to Y-axis up being negative
    m_Drivetrain.tankDrive(-driverLeftY, -driverRightY);
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
