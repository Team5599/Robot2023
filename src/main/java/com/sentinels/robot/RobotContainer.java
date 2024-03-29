/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot;

import com.sentinels.robot.constants.Ports;
import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.commands.armmech.arm.*;
import com.sentinels.robot.commands.armmech.intake.*;
import com.sentinels.robot.commands.autonomous.*;
import com.sentinels.robot.commands.drivetrain.*;

import com.sentinels.robot.subsystems.arm.*;
import com.sentinels.robot.subsystems.drive.*;
import com.sentinels.robot.subsystems.intake.Intake;
import com.sentinels.robot.subsystems.vision.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems and commands are defined here.

  // Subsystems
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Limelight limelight = new Limelight();
  
  // Input Devices
  private final CommandXboxController driver = new CommandXboxController(Ports.Controllers.DRIVER);
  private final CommandJoystick operator = new CommandJoystick(Ports.Controllers.OPERATOR);

  // Autonomous
  private static SendableChooser<Command> autonChooser = new SendableChooser<>();
  public static SendableChooser<Boolean> controlChooser = new SendableChooser<>();
  public static double distance;

  private Boolean arcadeDriveActive;

  public RobotContainer() {
    
    configureButtonBindings();
    configureDefaultCommands();
    configureAutonCommands();

    DriverStation.silenceJoystickConnectionWarning(true);
  }

  // BUTTON BINDINGS

  private void configureButtonBindings() {
    configureDriverBindings();
    configureOperatorBindings();
    // configureReset()
  }

  private void configureDriverBindings() {
    controlChooser.setDefaultOption("Tank Drive", false);
    controlChooser.addOption("Tank Drive", false);
    controlChooser.addOption("Arcade Drive", true);

    SmartDashboard.putData("Control Scheme", controlChooser);
    arcadeDriveActive = controlChooser.getSelected();
  }

  private void configureOperatorBindings() {
    // ARM
    operator.povUp().whileTrue(new RepeatCommand(new ArmCascade(arm, operator)));
    operator.povDown().whileTrue(new RepeatCommand(new ArmCascade(arm, operator)));

    operator.axisLessThan(1, -0.12).whileTrue(new ArmPivot(arm, operator));
    operator.axisGreaterThan(1, 0.12).whileTrue(new ArmPivot(arm, operator));

    // INTAKE
    operator.button(3).onTrue(new IntakeOpen(intake));
    operator.button(5).onTrue(new IntakeClose(intake));
    
    operator.button(6).whileTrue(new RepeatCommand(new IntakePivot(intake, operator)));
    operator.button(4).whileTrue(new RepeatCommand(new IntakePivot(intake, operator)));

    //RESET
    operator.button(4).and(operator.button(6)).whileTrue(new ResetArm(arm, operator));
    
  }
  // COMMAND DEFAULTS

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver, arcadeDriveActive));
    arm.setDefaultCommand(new ArmPivot(arm, operator));
    //intake.setDefaultCommand(new IntakeStall(intake));
  }

  private void configureAutonCommands() {
    // SIMULATION TESTS
    autonChooser.addOption("PID test", Autos.PIDtest(drivetrain, limelight));
    // autonChooser.addOption("Trajectory test", Autos.RamseteDrive(drivetrain, Arena.Trajectories.SimpleTrajectory, true));

    // TESTING
    autonChooser.addOption("Arm Angle Test", Autos.ArmAngleTest(drivetrain, arm, intake));
    autonChooser.addOption("Dock Test", Autos.DockingTest(drivetrain));

    // REAL
    autonChooser.addOption("Disabled", null);
    autonChooser.addOption("Leave Community", Autos.SimpleTimedDrive(drivetrain, Settings.Drivetrain.kTimedDriveTime));
    autonChooser.addOption("Score & Exit", Autos.BasicAuton(drivetrain, arm, intake));

    autonChooser.setDefaultOption("Score & Exit", Autos.BasicAuton(drivetrain, arm, intake));
    
    SmartDashboard.putData("Selected Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}