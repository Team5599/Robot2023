/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.commands.armmech.intake;

import com.sentinels.robot.subsystems.intake.Intake;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class IntakePivot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private Intake intake;
  private CommandJoystick operator;
  private SlewRateLimiter limiter = new SlewRateLimiter(0.5);

  public IntakePivot(Intake intake, CommandJoystick operator) {
    this.intake = intake;
    this.operator = operator;
    
    addRequirements(intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    //THIS GOT INVERTED AND UNINVERTED MULTIPLE TIMES
    if (operator.button(6).getAsBoolean()) {
      intake.intakePivot(limiter.calculate(0.10));
    } else {
      intake.intakePivot(limiter.calculate(-0.10));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.pivotStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
