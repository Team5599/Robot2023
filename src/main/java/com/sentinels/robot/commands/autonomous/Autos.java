// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

// Import all subsystems for auton use
//import com.sentinels.robot.subsystems.*;

public final class Autos {
  
  /** Example static factory for an autonomous command. */
  public static CommandBase autonomous(Object subsystem) {
    return Commands.sequence();
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}