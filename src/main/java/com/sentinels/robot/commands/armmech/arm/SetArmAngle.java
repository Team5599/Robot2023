// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmAngle extends CommandBase {

  private final Arm arm;
  private double targetAngle;
  private double currentAngle;
  private double ACCEPTABLE_ERROR = 2.0;

  //input is a real world angle relative to the floor
  public SetArmAngle(Arm arm, double angleOffet) {

    this.arm = arm;
    this.currentAngle = arm.getArmPivotAngle();

    // The following ensures that the target angle will always be a number between 0 and 360
    if (currentAngle + angleOffet < 0) {
      // If the target angle is less than 0, convert this to the equivalent number counting backwards from 360
      // eg. -50degrees is 310deg
      // eg. -400deg is 320deg
      this.targetAngle = 360 - (Math.abs((currentAngle + angleOffet) % 360));
    } else {
      // eg. 60deg is 60deg
      // eg. 400deg is 40deg
      this.targetAngle = (currentAngle + angleOffet) % 360;
    } 

    addRequirements(arm);
  }

  // public SetArmAngle(Arm arm, level targetAngle) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   this.arm = arm;
  //   this.targetAngle = targetAngle;
  //   addRequirements(arm);
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //conversion to in code angle
    // targetAngle-=Settings.Arm.kPivotStartingAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double angleDifference = targetAngle - currentAngle;

    SmartDashboard.putNumber("Arm/Cascade Auto/Target Angle", targetAngle);
    SmartDashboard.putNumber("Arm/Cascade Auto/Current Angle", currentAngle);
    SmartDashboard.putNumber("Arm/Cascade Auto/Angle Difference", angleDifference);
    SmartDashboard.putNumber("Arm/Cascade Auto/Encoder Value Actual", arm.getLeftPosition());

    if (isWithinBounds(currentAngle, targetAngle)) return;

    System.out.println("Moving arm . . . [" + currentAngle + " | " + targetAngle + " | " + angleDifference + "]");

    if (targetAngle > currentAngle){
      arm.PivotArm(0.15);
    } else {
      arm.PivotArm(-0.15);
    }
  }

  public boolean isWithinBounds(double angle0, double angle1) {
    if (Math.abs(angle1 - angle0) < ACCEPTABLE_ERROR) {
      System.out.println("Arm is within acceptable bounds [" + angle1 + " |" + angle0 + "]");
      return true;
    }
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWithinBounds(targetAngle, currentAngle);
  }
}
