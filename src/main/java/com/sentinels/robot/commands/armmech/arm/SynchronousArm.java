// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import com.sentinels.robot.constants.Settings.Arm.Level;
import com.sentinels.robot.subsystems.arm.Arm;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SynchronousArm extends ParallelCommandGroup {

  //TODO: add a command for controlling the intake's pivot
  public SynchronousArm(Arm arm, double length, double angle) {
    addCommands(new PivotPID(arm, angle), new CascadePID(arm, length));
  }

  public SynchronousArm(Arm arm, Level level) {
    addCommands(new PivotPID(arm, level), new CascadePID(arm, level));
  }
}
