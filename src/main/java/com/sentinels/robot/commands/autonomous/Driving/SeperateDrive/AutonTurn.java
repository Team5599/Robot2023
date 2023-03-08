// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving.SeperateDrive;

import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonTurn extends CommandBase {
  /** Creates a new AutonTurn. */
  private final Drivetrain drivetrain;
  private final Limelight limelight;
  //private final IMU imu;

  private final PIDController leftController;
  private final PIDController rightController;
  private double wheelDistance;
  double limelightAngle;// the angle from us to the game piece
  public AutonTurn(Drivetrain drivetrain, Limelight limelight, IMU imu) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    //this.imu = imu;
    leftController = new PIDController(0.3, 0.1, 0.1);
    rightController = new PIDController(0.3, 0.1, 0.1);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override

  public void initialize() {
    wheelDistance = (Units.degreesToRadians(limelightAngle))*((Settings.Drivetrain.kWheelTrackWidth)/2);
    leftController.reset();
    rightController.reset();
    leftController.setSetpoint(-wheelDistance);
    rightController.setSetpoint(wheelDistance);
    
    //double radiusW = 3.0; //radius of the wheel
    //double halfWidthR = 13.5;//half of width of the robot
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.voltageDrive(leftController.calculate(drivetrain.getLeftPosition()), drivetrain.getRightPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveStop();
    //andThen(new AutonDriveDistance(drivetrain, limelight));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(leftController.atSetpoint() && rightController.atSetpoint()){
      return true;
    }
    return false;
  }
}
