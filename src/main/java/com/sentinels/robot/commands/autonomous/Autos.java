// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

//import com.sentinels.robot.constants.Ports.Arm;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;
import com.sentinels.robot.commands.autonomous.Driving.AutonDock;
import com.sentinels.robot.commands.autonomous.Driving.SeperateDrive.AutonDriveDistance;
import com.sentinels.robot.commands.drivetrain.DrivetrainVoltageDrive;
import com.sentinels.robot.subsystems.arm.Arm;
import com.sentinels.robot.subsystems.intake.ArmIntake;
import com.sentinels.robot.constants.Arena;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



// Import all subsystems for auton use
//import com.sentinels.robot.subsystems.*;

public final class Autos {
  /** Example static factory for an autonomous command. */

  private final Drivetrain drivetrain;
  private final Arm arm;
  private final ArmIntake intake;
  private final IMU imu;
  private final Limelight limelight;

  public static CommandBase autonomous(Drivetrain drivetrain, Arm arm, ArmIntake intake, IMU imu, Limelight limelight) {
    //TODO: does not work with 1.5, works with 1.8 though. setpoints can only be multiples of 0.45
    return Commands.sequence(
      new AutonDriveDistance(drivetrain, limelight, 4.5, false)
    );
  }
  //TODO: ramsete controller oftenly has insane voltage spikes, find a way to stop them to keep the robot safe
  public static CommandBase RamseteTest(Drivetrain drivetrain, Arm arm, IMU imu, Limelight limelight){
    return Commands.sequence(
      new RamseteCommand(
        Arena.Trajectories.SimpleTrajectory, 
        drivetrain::getPose, 
        new RamseteController(0.2,0.5), //b and zeta, not sure what they are tbh
        new SimpleMotorFeedforward(2, 2, 2),//voltages here, arbitrary numbers here for now
        Settings.Drivetrain.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        new PIDController(2, 0, 0),//both of these are arbitrary, set these later 
        new PIDController(2, 0, 0), 
        drivetrain::voltageDrive,
        drivetrain
      ).andThen(
        () -> drivetrain.voltageDrive(0, 0)
      )
    );
  }

  public static CommandBase voltageTest(Drivetrain drivetrain){
    return Commands.sequence(
      new DrivetrainVoltageDrive(drivetrain)
    );
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}