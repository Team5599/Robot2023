// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.drivetrain;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private final SlewRateLimiter limiterL = new SlewRateLimiter(0.6);
  private final SlewRateLimiter limiterR = new SlewRateLimiter(0.6);

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
    if(RobotBase.isSimulation()){
      driverLeftY = driver.getLeftY();
      driverLeftX = driver.getRightY();
    }

    periodic();
    if (arcadeDriveActive) {
      drivetrain.arcadeDrive(driverRightY * Settings.Drivetrain.kDriveSpeedCap, driverLeftX * Settings.Drivetrain.kDriveSpeedCap);
    }
    else {
      drivetrain.tankDrive(driverLeftY * Settings.Drivetrain.kDriveSpeedCap, driverRightY * Settings.Drivetrain.kDriveSpeedCap);
    }
  }

  public void periodic(){
    double[] axes = {driverLeftY,driverRightY};
    SmartDashboard.putNumberArray("Controller/Inputs (LY,RY):", axes);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
