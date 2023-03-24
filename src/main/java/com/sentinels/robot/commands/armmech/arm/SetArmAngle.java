// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.constants.Settings.Arm.level;
import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetArmAngle extends CommandBase {

  private final Arm arm;
  private double targetAngle;
  private double currentAngle;
  private double ACCEPTABLE_ERROR = 2.0;

  //input is a real world angle relative to the floor
  public SetArmAngle(Arm arm, double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.targetAngle = targetAngle;
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
    currentAngle = arm.getArmPivotAngle();
    double angleDifference = targetAngle - currentAngle;

    SmartDashboard.putNumber("Arm/Cascade Auto/Target Angle", targetAngle);
    // SmartDashboard.putNumber("Arm/Cascade Auto/Current Angle", currentAngle);
    SmartDashboard.putNumber("Arm/Cascade Auto/Angle Difference", angleDifference);
    // SmartDashboard.putNumber("Arm/Cascade Auto/Encoder Value Actual", arm.getArmEncoderValue());

    if (isWithinBounds(currentAngle, targetAngle)) return;

    System.out.println("Moving arm . . . [" + currentAngle + " | " + targetAngle + " | " + angleDifference + "]");

    if (targetAngle > currentAngle){
      arm.PivotArm(0.1);
    } else {
      arm.PivotArm(-0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  public boolean isWithinBounds(double angle0, double angle1) {
    if (Math.abs(angle1 - angle0) < ACCEPTABLE_ERROR) {
      System.out.println("Arm is within acceptable bounds [" + angle1 + " |" + angle0 + "]");
      return true;
    };
    return false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isWithinBounds(targetAngle, currentAngle);
  }
}
