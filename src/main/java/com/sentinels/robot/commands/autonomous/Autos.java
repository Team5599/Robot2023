// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

//import com.sentinels.robot.constants.Ports.Arm;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;
import com.sentinels.robot.Robot;
import com.sentinels.robot.constants.Settings;
import com.sentinels.robot.subsystems.arm.Arm;
import com.sentinels.robot.subsystems.intake.ArmIntake;

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
    return Commands.sequence(
      
    );
  }
  public static CommandBase RamseteTest(Drivetrain drivetrain, Arm arm, IMU imu, Limelight limelight){
    
    return Commands.sequence(
      /*new RamseteCommand(
        Robot.trajectory, 
        drivetrain::getPose, 
        new RamseteController(0,0), //b and zeta, not sure what they are tbh
        new SimpleMotorFeedforward(0, 0),//voltages here 
        Settings.Drivetrain.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        new PIDController(0, 0, 0), 
        new PIDController(0, 0, 0), 
        null, 
        drivetrain
      )*/

    );
  }

  public void setRamsete(){

  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}