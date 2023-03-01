/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

package com.sentinels.robot;

import com.sentinels.robot.constants.Ports;
//command imports
import com.sentinels.robot.commands.armmech.arm.*;
import com.sentinels.robot.commands.armmech.intake.*;
import com.sentinels.robot.commands.autonomous.*;
import com.sentinels.robot.commands.autonomous.Driving.AutonDock;
import com.sentinels.robot.commands.autonomous.Driving.SeperateDrive.AutonDriveDistance;
import com.sentinels.robot.commands.autonomous.Driving.SeperateDrive.AutonTurn;
import com.sentinels.robot.commands.drivetrain.*;
//subsystem imports
import com.sentinels.robot.subsystems.arm.*;
import com.sentinels.robot.subsystems.drive.*;
import com.sentinels.robot.subsystems.intake.ArmIntake;
import com.sentinels.robot.subsystems.odometry.*;
import com.sentinels.robot.subsystems.vision.*;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  // The robot's subsystems and commands are defined here.

  // Subsystems
  private final Arm arm = new Arm();
  private final ArmIntake armIntake = new ArmIntake();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Camera camera = new Camera();
  private final Limelight limelight = new Limelight();
  private final IMU imu = new IMU();
  
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
  }

  private void configureDriverBindings() {
    controlChooser.setDefaultOption("Tank Drive", false);

    controlChooser.addOption("Tank Drive", false);
    controlChooser.addOption("Arcade Drive", true);
    SmartDashboard.putData("Control Scheme", controlChooser);

    arcadeDriveActive = controlChooser.getSelected();

    driver.b().whileTrue(new DrivetrainStop(drivetrain));
  }

  private void configureOperatorBindings() {
    operator.axisGreaterThan(2, 0).whileTrue(new ArmPivot(arm, operator));//pulley pivoting
    operator.axisLessThan(2, 0).whileTrue(new ArmPivot(arm, operator));

    operator.axisGreaterThan(4, 0).whileTrue(new ArmExtend(arm, operator));//arm retraction and extension
    operator.axisLessThan(4,0).whileTrue(new ArmExtend(arm, operator));

    operator.button(1).whileTrue(new IntakeOpen(armIntake));
    
    //operator.button(5).whileTrue(null);// intake command calls
    //operator.button(4).whileTrue(null);//

    // HEIGHT PRESETS
    
  }

  // COMMAND DEFAULTS

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver, arcadeDriveActive));
  }

  private void configureAutonCommands() {
    autonChooser.addOption("Disabled", null);
    autonChooser.addOption("Auton test", Autos.autonomous(drivetrain, arm, armIntake, imu, limelight));
    autonChooser.addOption("Ramsete test", Autos.RamseteTest(drivetrain, arm, imu, limelight));
    autonChooser.addOption("voltage drive test", Autos.voltageTest(drivetrain));
    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    return autonChooser.getSelected();
  }
}