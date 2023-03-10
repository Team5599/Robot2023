// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PivotPID extends CommandBase {
  private final Arm arm;
  private final PIDController pivotController;

  public PivotPID(Arm arm, double angle) {
    this.arm = arm;
    pivotController = new PIDController(0.5,0,0);
    pivotController.setSetpoint(27*angle);//using 27 here because thats the gear ratio
    addRequirements(arm);
  }

  // need an overload for an enum

  @Override
  public void initialize() {
    pivotController.reset();
  }

  @Override
  public void execute() {   
    //TODO: there needs to be a way to detect the start position and also a way to callibrate everything properly
    // distance may have to be negated
    // if the pivot starts at 0 degrees, then we need to choose the bottom position to be the actual max amount of degrees, but the input should still be normal
    // input should be from 0 to `70 degrees but map to the proper places
    //arm outer length: 47", arm inner length: 40", base length: 28.5"
    arm.PivotArm(pivotController.calculate(arm.getRightPosition()));

    //SmartDashboard.putNumber("Arm/PID",pivotController.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pivotController.atSetpoint() == true){
      return true;
    }
    return false;
  }
}
