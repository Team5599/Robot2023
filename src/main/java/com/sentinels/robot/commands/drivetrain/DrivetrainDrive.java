// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.drivetrain;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class DrivetrainDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Drivetrain m_Drivetrain;
  private final CommandXboxController driver;

  double driverLeftY;
  double driverRightY;

  private final SlewRateLimiter limiter = new SlewRateLimiter(0.4);

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
    driverLeftY = limiter.calculate(driver.getLeftY());
    driverRightY = limiter.calculate(driver.getRightY());

    m_Drivetrain.tankDrive(driverLeftY * Settings.Drivetrain.DRIVESPEEDCAP, driverRightY * Settings.Drivetrain.DRIVESPEEDCAP);
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
