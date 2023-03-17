// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

//import com.sentinels.robot.constants.Ports.Arm;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;
import com.sentinels.robot.commands.autonomous.Driving.AutonDock;
import com.sentinels.robot.commands.autonomous.Driving.AutonDriveDistance;
import com.sentinels.robot.commands.drivetrain.DrivetrainVoltageDrive;
import com.sentinels.robot.subsystems.arm.Arm;
import com.sentinels.robot.subsystems.intake.ArmIntake;
import com.sentinels.robot.constants.Arena;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;



// Import all subsystems for auton use
//import com.sentinels.robot.subsystems.*;

public final class Autos {
  /** Example static factory for an autonomous command. */

  // private final Drivetrain drivetrain;
  // private final Arm arm;
  // private final ArmIntake intake;
  // private final IMU imu;
  // private final Limelight limelight;


  public static CommandBase Routine0(Drivetrain drivetrain, Arm arm, ArmIntake intake, Limelight limelight){

    RamseteController disabled = new RamseteController(0.2, 0.5);
    disabled.setEnabled(false);

    PIDController leftController = new PIDController(3, 0, 0);
    PIDController rightController = new PIDController(3, 0, 0);

    //Trajectory traj = new Trajectory();

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


  public static CommandBase PIDtest(Drivetrain drivetrain, IMU imu, Limelight limelight) {
    //TODO: does not work with 1.5, works with 1.8 though. setpoints can only be multiples of 0.45
    return Commands.sequence(
      new AutonDriveDistance(drivetrain, limelight, 4.5 , false)
    );
  }

  //https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/troubleshooting.html
  public static CommandBase RamseteTest(Drivetrain drivetrain, IMU imu, Limelight limelight){
    
    var RamseteControl = NetworkTableInstance.getDefault().getTable("Ramsete Control");
    var PIDleftSetpoint = RamseteControl.getEntry("Left setpoint");
    var leftVel = RamseteControl.getEntry("Left Velocity");
    var PIDrightSetpoint = RamseteControl.getEntry("Right setpoint");
    var rightVel = RamseteControl.getEntry("Right Velocity");

    RamseteController disabled = new RamseteController(0.2, 0.5);
    disabled.setEnabled(false);

    PIDController leftController = new PIDController(3, 0, 0);
    PIDController rightController = new PIDController(3, 0, 0);

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
        // drivetrain::voltageDrive,1
        (left, right)->{
          drivetrain.voltageDrive(left, right);

          leftVel.setNumber(drivetrain.getLeftVelocity());
          PIDleftSetpoint.setNumber(leftController.getSetpoint());
          rightVel.setNumber(drivetrain.getRightVelocity());
          PIDrightSetpoint.setNumber(rightController.getSetpoint());

          SmartDashboard.putNumber("RAMSETE/Left Velocity",drivetrain.getLeftVelocity());
          SmartDashboard.putNumber("RAMSETE/Right Velocity",drivetrain.getRightVelocity());
          SmartDashboard.putNumber("RAMSETE/Left Setpoint", leftController.getSetpoint());
          SmartDashboard.putNumber("RAMSETE/Right Setpoint", rightController.getSetpoint());

          SmartDashboard.putNumber("RAMSETE/Left Position Error", leftController.getPositionError());          
          SmartDashboard.putNumber("RAMSETE/Left Velocity Error", leftController.getVelocityError());
          SmartDashboard.putNumber("RAMSETE/Right Position Error", rightController.getPositionError());          
          SmartDashboard.putNumber("RAMSETE/Right Velocity Error", rightController.getVelocityError());
        },
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