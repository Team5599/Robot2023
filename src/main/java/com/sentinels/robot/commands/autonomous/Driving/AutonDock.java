// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDock extends CommandBase {
  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final IMU imu;

  /** Creates a new AutonDock. */
  public AutonDock(Drivetrain drivetrain, Limelight limelight, IMU imu) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.imu = imu;
    addRequirements(drivetrain);// might need the IMU in here
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
