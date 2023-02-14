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
import com.sentinels.robot.commands.drivetrain.*;

import com.sentinels.robot.subsystems.arm.*;
import com.sentinels.robot.subsystems.drive.*;
import com.sentinels.robot.subsystems.odometry.*;
import com.sentinels.robot.subsystems.vision.*;

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
    driver.b().whileTrue(new DrivetrainStop(drivetrain));
  }

  private void configureOperatorBindings() {
    operator.axisGreaterThan(2, 0).whileTrue(new ArmPivot(arm, operator));//pulley pivoting
    operator.axisLessThan(2, 0).whileTrue(new ArmPivot(arm, operator));

    operator.axisGreaterThan(4, 0).whileTrue(new ArmExtend(arm, operator));//arm retraction and extension
    operator.axisLessThan(4,0).whileTrue(new ArmExtend(arm, operator));

    operator.button(1).whileTrue(new IntakeOpen(armIntake));
  }

  // COMMAND DEFAULTS

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(new DrivetrainDrive(drivetrain, driver));
  }

  private void configureAutonCommands() {
    autonChooser.addOption("Disabled", null);

    SmartDashboard.putData("Autonomous", autonChooser);
  }

  public Command getAutonomousCommand() {
    /*
     * the sequential command group will have a list of all the auto commands we will need
     * 
     * auto commands:
     * DrivePID(after findout the target location and the current location using the april tags, make the robot drive close enought ot the game pieces)
     * IntakeAUTO(pick up and raise the game piece)
     * DepositAUTO(place the game pice in the middle or high row)
     * DrivePID(this time drive to the charging dock)
     * BalanceAUTO(using the IMU, stay engaged on the charging dock while it tilts)
     */
     return autonChooser.getSelected();
  }
}
