// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Constants
import frc.robot.Constants.OperatorConstants;

// Commands
import frc.robot.commands.armmech.arm.*;
import frc.robot.commands.armmech.intake.*;
import frc.robot.commands.autonomous.*;
import frc.robot.commands.drivetrain.*;

// Subsystems
import frc.robot.subsystems.arm.*;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.odometry.*;
import frc.robot.subsystems.vision.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here.

  // Commands
  private final ArmExtend comArmExtend = new ArmExtend(null);
  private final ArmPivot comArmPivot = new ArmPivot(null);
  private final ArmRetract comArmRetract = new ArmRetract(null);
  private final IntakeClose comIntakeClose = new IntakeClose(null);
  private final IntakeOpen comIntakeOpen = new IntakeOpen(null);
  //private final Autos comAutos;
  private final DrivetrainDrive comDrivetrainDrive = new DrivetrainDrive(null);
  private final DrivetrainStop comDrivetrainStop = new DrivetrainStop(null);

  // Subsystems
  private final Arm m_Arm = new Arm();
  private final ArmIntake m_ArmIntake = new ArmIntake();
  private final Drivetrain m_Drivetrain = new Drivetrain();
  private final Camera m_Camera = new Camera();

  // Input Devices
  private final CommandXboxController driver = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandJoystick operator = new CommandJoystick(OperatorConstants.kOperatorJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    */

    // Placeholder triggers, delete if button not needed/used in the future
    driver.a().whileTrue(null);
    driver.b().whileTrue(null);
    driver.x().whileTrue(null);
    driver.y().whileTrue(null);
    driver.leftStick().whileTrue(null);
    driver.rightStick().whileTrue(null);
    driver.leftTrigger().whileTrue(
      new DrivetrainDrive(m_Drivetrain)
    );
    driver.rightTrigger().whileTrue(
      new DrivetrainDrive(m_Drivetrain)
    );
    driver.back().whileTrue(null); // select
    driver.start().whileTrue(null); // start
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(null);
  }
}
