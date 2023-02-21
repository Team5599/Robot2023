// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving.SeperateDrive;

import com.sentinels.robot.RobotContainer;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.vision.Camera;
import com.sentinels.robot.subsystems.vision.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonDriveDistance extends CommandBase {
  private final Drivetrain drivetrain;
  private final Limelight limelight;

  private final PIDController distanceController;

  private double setpoint;
  private boolean parallaxEnable;

  private double angle1;
  private double angle2;
  
  private double distance;// distance to gamepiece
  //Links for tmrw
  // https://docs.limelightvision.io/en/latest/cs_aimandrange.html
  // https://docs.limelightvision.io/en/latest/getting_started.html
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html


  //can also handle distance measurement with parallax

  public AutonDriveDistance(Drivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    distanceController = new PIDController(2,0.4, 0.4);
    parallaxEnable = false;
    addRequirements(drivetrain);
  }

  // overload for setpoint and distance measurement, can also be used for parallax
  public AutonDriveDistance(Drivetrain drivetrain, Limelight limelight, double setpoint, boolean parallaxEnable) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.parallaxEnable = parallaxEnable;
    distanceController = new PIDController(2,0.4, 0.4);
    distanceController.setSetpoint(setpoint);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceController.reset();
    if (parallaxEnable == false){ 
      setpoint = RobotContainer.distance-1;
    }
    else{
      angle1 = limelight.getTx();
    }
    distanceController.setSetpoint(setpoint);//stop somewhere right before the gamepiece for intake, arbitrary for now
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.tankDrive(
      distanceController.calculate(drivetrain.getLeftPosition(),setpoint), 
      distanceController.calculate(drivetrain.getRightPosition(),setpoint)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveStop();
    if(parallaxEnable == true){
      angle2 = limelight.getTx();
      RobotContainer.distance = limelight.ParllaxDistance(angle1, angle2);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(distanceController.atSetpoint()){
      return true;
    }
    return false;
  }
}
