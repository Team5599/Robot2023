/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.commands.drivetrain;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
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

  private final SlewRateLimiter limiterL = new SlewRateLimiter(3);
  private final SlewRateLimiter limiterR = new SlewRateLimiter(3);

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
    driverLeftX = 0;
  }

  @Override
  public void execute() {
    driverLeftY = limiterL.calculate(driver.getLeftY());
    driverRightY = limiterR.calculate(driver.getRightY());

    if (RobotBase.isSimulation()) {
      driverLeftY = driver.getLeftY();
      driverRightY = driver.getRightY();
    }

    if (arcadeDriveActive) {
      drivetrain.arcadeDrive(driverRightY * Settings.Drivetrain.kDriveSpeedCap, driverLeftX * Settings.Drivetrain.kDriveSpeedCap);
    } else {
      drivetrain.tankDrive(driverLeftY * Settings.Drivetrain.kDriveSpeedCap, driverRightY * Settings.Drivetrain.kDriveSpeedCap);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
