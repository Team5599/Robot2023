/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot;

import com.sentinels.robot.constants.Ports;

import com.sentinels.robot.commands.armmech.arm.*;
import com.sentinels.robot.commands.armmech.intake.*;
import com.sentinels.robot.commands.autonomous.*;
import com.sentinels.robot.commands.autonomous.Driving.AutonDock;
import com.sentinels.robot.commands.autonomous.Driving.AutonDriveDistance;
import com.sentinels.robot.commands.autonomous.Driving.AutonTurn;
import com.sentinels.robot.commands.drivetrain.*;

import com.sentinels.robot.subsystems.arm.*;
import com.sentinels.robot.subsystems.drive.*;
import com.sentinels.robot.subsystems.intake.Intake;
import com.sentinels.robot.subsystems.odometry.*;
import com.sentinels.robot.subsystems.vision.*;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here.

  // Subsystems
  private final Arm arm = new Arm();
  private final Intake armIntake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Camera camera = new Camera();
  private final Limelight limelight = new Limelight();
  private final IMU imu = new IMU();
  
  // Input Devices
  private final CommandXboxController driver = new CommandXboxController(Ports.Controllers.DRIVER);
  private final CommandXboxController operator = new CommandXboxController(Ports.Controllers.OPERATOR);

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
  }

  private void configureDriverBindings() {
    controlChooser.setDefaultOption("Tank Drive", false);
    controlChooser.addOption("Tank Drive", false);
    controlChooser.addOption("Arcade Drive", true);

    SmartDashboard.putData("Control Scheme", controlChooser);

    arcadeDriveActive = controlChooser.getSelected();

    //driver.b().whileTrue(new DrivetrainStop(drivetrain)); commented until drive team confirmation
  }

  private void configureOperatorBindings() {
    // ARM
    operator.leftTrigger().whileTrue(new ArmCascade(arm, operator));
    operator.rightTrigger().whileTrue(new ArmCascade(arm, operator));

    operator.leftStick().whileTrue(new ArmPivot(arm, operator));
    operator.rightStick().whileTrue(new ArmPivot(arm, operator));

    // INTAKE
    operator.a().onTrue(new IntakeOpen(armIntake));
    operator.b().onTrue(new IntakeClose(armIntake));

    // GAME PIECE MODES
    operator.x();
    operator.y();
  }

  // COMMAND DEFAULTS

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver, arcadeDriveActive));
  }

  private void configureAutonCommands() {
    autonChooser.addOption("Disabled", null);

    // SIMULATION TESTS

    autonChooser.addOption("PID test", Autos.PIDtest(drivetrain, imu, limelight));
    autonChooser.addOption("Ramsete test", Autos.RamseteTest(drivetrain, imu, limelight));
    autonChooser.addOption("voltage drive test", Autos.voltageTest(drivetrain));

    // REAL 

    //autonChooser.addOption("Routine 0", getAutonomousCommand());

    autonChooser.setDefaultOption("Ramsete test", Autos.RamseteTest(drivetrain, imu, limelight));
    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}