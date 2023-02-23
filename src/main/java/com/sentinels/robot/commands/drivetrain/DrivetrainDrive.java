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

  private final Drivetrain drivetrain;
  private final CommandXboxController driver;

  private double driverLeftY;
  private double driverRightY;
  private double driverLeftX;
  private Boolean arcadeDriveActive;

  private final SlewRateLimiter limiter = new SlewRateLimiter(0.4);

  public DrivetrainDrive(Drivetrain drivetrain, CommandXboxController controller, Boolean arcadeDriveActive) {
    this.drivetrain = drivetrain;
    this.driver = controller;
    this.arcadeDriveActive = arcadeDriveActive;

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
    driverLeftX = limiter.calculate(driver.getLeftX());

    if (arcadeDriveActive) {
      drivetrain.arcadeDrive(driverRightY * Settings.Drivetrain.kDriveSpeedCap, driverLeftX * Settings.Drivetrain.kDriveSpeedCap);
    }
    else {
      drivetrain.tankDrive(driverLeftY * Settings.Drivetrain.kDriveSpeedCap, driverRightY * Settings.Drivetrain.kDriveSpeedCap);
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
