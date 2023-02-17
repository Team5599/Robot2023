// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving.SeperateDrive;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonTurn extends CommandBase {
  /** Creates a new AutonTurn. */
  private final Drivetrain drivetrain;
  private final Limelight limelight;
  //private final IMU imu;

  private final PIDController directionController;
  double SetAngle;// the angle from us to the game piece
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public AutonTurn(Drivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    //this.imu = imu;
    directionController = new PIDController(0.3, 0.1, 0.1);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    directionController.reset();
    SetAngle = limelight.getTx(); 
    directionController.setSetpoint(SetAngle);
    
    //double radiusW = 3.0; //radius of the wheel
    //double halfWidthR = 13.5;//half of width of the robot
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double motorSpeed = directionController.calculate(limelight.getTx());
    drivetrain.tankDrive(motorSpeed, -motorSpeed); //does this give you the speed its going?
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
    if(directionController.atSetpoint()){
      return true;
    }
    return false;
  }
}
