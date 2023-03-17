// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

//import com.sentinels.robot.constants.Ports.Arm;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.vision.Limelight;
import com.sentinels.robot.Robot;
import com.sentinels.robot.commands.autonomous.Driving.AutonDock;
import com.sentinels.robot.commands.autonomous.Driving.SeperateDrive.AutonDriveDistance;
import com.sentinels.robot.commands.drivetrain.DrivetrainVoltageDrive;
import com.sentinels.robot.subsystems.arm.Arm;
import com.sentinels.robot.constants.Arena;
import com.sentinels.robot.subsystems.intake.Intake;
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
  private final Intake intake;
  private final Limelight limelight;

  public static CommandBase autonomous(Drivetrain drivetrain, Arm arm, Intake intake, Limelight limelight) {
    return Commands.sequence(
      new AutonDriveDistance(drivetrain, limelight, 1.5 , false)
    );
  }

  //theres an error here that doesnt let me simulate the code

  public static CommandBase RamseteTest(Drivetrain drivetrain, Arm arm, IMU imu, Limelight limelight){
    return Commands.sequence(
      new RamseteCommand(
        Arena.Trajectories.TestTrajectory, 
        drivetrain::getPose, 
        new RamseteController(0.5,0.5), //b and zeta, not sure what they are tbh
        new SimpleMotorFeedforward(5, 5, 4),//voltages here, arbitrary numbers here for now
        Settings.Drivetrain.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        new PIDController(2, 0, 0),//both of these are arbitrary, set these later 
        new PIDController(2, 0, 0), 
        //drivetrain::voltageDrive, 
        drivetrain::tankDrive,
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