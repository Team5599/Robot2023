/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.commands.armmech.intake;

import com.sentinels.robot.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class IntakePivot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Intake intake;
  private CommandJoystick operator;

  public IntakePivot(Intake intake, CommandJoystick operator) {
    this.intake = intake;
    
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // TODO: Put proper axis number in the getRawAxis() method below
    intake.intakePivot(-operator.getRawAxis(4));
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
