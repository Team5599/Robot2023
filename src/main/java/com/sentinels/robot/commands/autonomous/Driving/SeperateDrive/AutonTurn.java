// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving.SeperateDrive;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.vision.Limelight;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonTurn extends CommandBase {
  /** Creates a new AutonTurn. */
  private final Drivetrain drivetrain;
  private final Limelight limelight;

  private final PIDController directionController;
  //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public AutonTurn(Drivetrain drivetrain, Limelight limelight, double xoffset) {
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    directionController = new PIDController(0.1, 0.1, 0.1);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    directionController.reset();
    
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    NetworkTableEntry xoffset = limelight.getTx(); //gets the angle offset of the gamepiece
    double radiusS = 3.0; //radius of the smaller wheel
    double distanceT; //distance between driveTrain wheels
/*
 * peudocode:
 * 
 * we first get the circumference of the entire robot using the distance between the two drivetrains as a radius
 * the xoffset/360 is proportional to distance_turn_to_get_degree / total_circumference
 * and once we have the distance_turn_to_get_degree, amount_of_rotations_per_smaller_wheel x 2pi(radius_of_smaller_wheel) = distance_turn_to_get_degree
 * so we could use that equation to solve for amount_of_rotations_per_smaller_wheel
*/
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
