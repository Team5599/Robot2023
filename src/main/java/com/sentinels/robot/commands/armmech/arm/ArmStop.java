/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Autonomous-Exclusive Command
 */
public class ArmStop extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final Arm arm;

  public ArmStop(Arm arm) {
    this.arm = arm;
    
    addRequirements(arm);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    arm.StopArm();
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
