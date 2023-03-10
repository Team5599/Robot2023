// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CascadePID extends CommandBase {
  private final Arm arm;
  private final PIDController cascadeController;

  //TODO:put this length into the constants file 
  private final double cascadeLength = 21; // in inches
  private final double gearRatio = 27;

  /**
   * @param arm the arm subsystem
   * @param input a number from 0 to 1, where 1 is full extended and 0 is fully retracted
   */
  public CascadePID(Arm arm, double input) {
    this.arm = arm;
    cascadeController = new PIDController(1, 1, 1);
    cascadeController.setSetpoint(input * cascadeLength * gearRatio);
  }

  // need an overload to take a string name/ enum / distance in meters

  // public CascadePID(Arm arm, double distance) {
  //   this.arm = arm;
  //   cascadeController = new PIDController(1, 1, 1);
  //   cascadeController.setSetpoint(distance * gearRatio);
  // }

  @Override
  public void initialize() {
    cascadeController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(cascadeController.atSetpoint() == true){
      return true;
    }
    return false;
  }
}
