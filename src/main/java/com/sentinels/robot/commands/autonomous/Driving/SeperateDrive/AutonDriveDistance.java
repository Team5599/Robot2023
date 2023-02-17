// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving.SeperateDrive;

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

  public double setpoint;
  //Links for tmrw
  // https://docs.limelightvision.io/en/latest/cs_aimandrange.html
  // https://docs.limelightvision.io/en/latest/getting_started.html
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/pidcontroller.html

  public AutonDriveDistance(Drivetrain drivetrain, Limelight limelight) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    distanceController = new PIDController(2,0.4, 0.4);
    //distanceController.setSetpoint(0);
    addRequirements(drivetrain);
  }
  // overload for setpoint and distance measurement
  public AutonDriveDistance(Drivetrain drivetrain, Limelight limelight, double setpoint) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    distanceController = new PIDController(2,0.4, 0.4);
    distanceController.setSetpoint(setpoint);
    addRequirements(drivetrain);
  }
  public double getSetpoint(){
    return limelight.getDistance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceController.reset();
    //System.out.println(distanceController.getSetpoint());
    if (!(distanceController.getSetpoint() == 0.0)){ //this will probably not work
      double distance = limelight.getDistance();
      setpoint = distance-1;
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
