// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving.SeperateDrive;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.vision.Camera;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDriveDistance extends CommandBase {
  private final Drivetrain drivetrain;
  private final Camera camera;

  //private final PIDController direction controller


  //Link for tmrw
  // https://docs.limelightvision.io/en/latest/cs_aimandrange.html

  public AutonDriveDistance(Drivetrain drivetrain, Camera camera) {
    this.drivetrain = drivetrain;
    this.camera = camera;
    addRequirements(drivetrain, camera);
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
