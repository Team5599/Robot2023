// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.subsystems.arm.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class pivotPID extends CommandBase {
  private final Arm arm;
  private final PIDController pivotController;



  //TODO: we need a command to callibrate the arm pivot motor
  public pivotPID(Arm arm, double setpoint) {
    this.arm = arm;
    pivotController = new PIDController(0, 0, 0);
    pivotController.setSetpoint(setpoint * 27);

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    pivotController.reset();
  }

  @Override
  public void execute() {
    arm.PivotArm(pivotController.calculate(arm.getRightPosition()));
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    if(pivotController.atSetpoint()){
      return true;
    }
    return false;
  }
}
