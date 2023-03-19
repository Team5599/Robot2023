// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.vision.Limelight;
import com.sentinels.robot.commands.autonomous.Driving.AutonDriveDistance;
import com.sentinels.robot.commands.drivetrain.DrivetrainVoltageDrive;
import com.sentinels.robot.subsystems.arm.Arm;
import com.sentinels.robot.subsystems.intake.Intake;
import com.sentinels.robot.constants.Arena;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {

  private static RamseteController disabled = new RamseteController(0.2, 0.5);
  private static PIDController leftController = new PIDController(3, 0, 0);
  private static PIDController rightController = new PIDController(3, 0, 0);

  public static CommandBase Routine0(Drivetrain drivetrain, Arm arm, Intake intake, Limelight limelight){    
    disabled.setEnabled(false);

    return Commands.sequence(
      new RamseteCommand(
        Arena.Trajectories.TestTrajectory, 
        drivetrain::getPose, 
        //Ramsete controlle original values: 0.2, 0.5
        // new RamseteController(0.1,0.2), //b and zeta (how much it turns), larger values = smaller turn (think of the turns as the size of a arc)
        disabled,
        new SimpleMotorFeedforward(0.15, 2, 2),//voltages here, arbitrary numbers here for now
        Settings.Drivetrain.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        leftController,
        rightController,
        drivetrain::voltageDrive
      )
      .andThen(
        () -> drivetrain.voltageDrive(0, 0)
      )
    );
  }


  public static CommandBase PIDtest(Drivetrain drivetrain, Limelight limelight) {
    //TODO: does not work with 1.5, works with 1.8 though. setpoints can only be multiples of 0.45
    return Commands.sequence(
      new AutonDriveDistance(drivetrain, limelight, 4.5 , false)
    );
  }

  public static CommandBase RamseteTest(Drivetrain drivetrain, Limelight limelight) {
    return Commands.sequence(
      new RamseteCommand(
        Arena.Trajectories.TestTrajectory, 
        drivetrain::getPose, 
        //Ramsete controlle original values: 0.2, 0.5
        // new RamseteController(0.1,0.2), //b and zeta (how much it turns), larger values = smaller turn (think of the turns as the size of a arc)
        disabled,
        new SimpleMotorFeedforward(0.15, 2, 2),//voltages here, arbitrary numbers here for now
        Settings.Drivetrain.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        leftController,
        rightController,
        drivetrain::voltageDrive
      )
      .andThen(
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