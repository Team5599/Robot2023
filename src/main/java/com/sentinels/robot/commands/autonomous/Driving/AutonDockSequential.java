// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving;

import com.sentinels.robot.RobotContainer;
import com.sentinels.robot.subsystems.drive.Drivetrain;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonDockSequential extends SequentialCommandGroup {
  /** Creates a new AutonDockSequential. */
  public AutonDockSequential(Drivetrain drivetrain) {
    addCommands(
      // new Auton
      new AutonDock(drivetrain)
      
    );
  }
}
