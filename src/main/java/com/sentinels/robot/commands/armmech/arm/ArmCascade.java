/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ArmCascade extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Arm arm;
  private final CommandJoystick operator;

  public ArmCascade(Arm arm, CommandJoystick operator) {
    this.arm = arm;
    this.operator = operator;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.CascadeArm(-operator.getRawAxis(6));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.StopArm();
  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
