// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.armmech.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import com.sentinels.robot.subsystems.arm.Arm;

/** An example command that uses an example subsystem. */
public class ArmPivot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Arm m_ArmSubsystem;

  double pulleySpeed;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArmPivot(Arm subsystem, CommandJoystick operator ) {
    m_ArmSubsystem = subsystem;
    pulleySpeed = operator.getRawAxis(2);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pulleySpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_ArmSubsystem.setPulley(pulleySpeed);
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
