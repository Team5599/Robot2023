// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous;

import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.vision.Limelight;
import com.sentinels.robot.commands.armmech.arm.SetArmAngle;
import com.sentinels.robot.commands.armmech.intake.IntakeOpen;
import com.sentinels.robot.commands.autonomous.Driving.AutonDriveDistance;
import com.sentinels.robot.commands.autonomous.Driving.AutonTimedDrive;
import com.sentinels.robot.subsystems.arm.Arm;
import com.sentinels.robot.subsystems.intake.Intake;
import com.sentinels.robot.constants.Settings;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class Autos {

  public static CommandBase SimpleTimedDrive(Drivetrain drivetrain, double seconds) {
    return Commands.sequence(
      new AutonTimedDrive(drivetrain, false).withTimeout(seconds)
    );
  }

  public static CommandBase ArmAngleTest(Drivetrain drivetrain, Arm arm, Intake intake) {
    arm.resetEncoders();
    return Commands.sequence(
      new SetArmAngle(arm, 25)
    );
  }

  public static CommandBase BasicAuton(Drivetrain drivetrain, Arm arm, Intake intake) {
    return Commands.sequence(
      // Lower the arm
      new SetArmAngle(arm, -5),
      // Drop the cube
      new IntakeOpen(intake),
      // Drive to community (backwards)
      new AutonTimedDrive(drivetrain, true).withTimeout(3.9),
      new SetArmAngle(arm, 15)
    );
  }

  /**
   * Command that fills in a Ramsete command based on the robot and pre-tuned values for PID, feedforward and Ramsete
   * @param drivetrain - the drivetrain subsystem
   * @param trajectory - the trajectory that the robot will drive
   * @param diagnostic - allow if the Ramsete graph is changed and if the path is displayed in a simulation
   * @return the command itself
   */
  public static CommandBase RamseteDrive(Drivetrain drivetrain, Trajectory trajectory, boolean displayStats) {
    var RamseteControl = NetworkTableInstance.getDefault().getTable("Ramsete Control");
    var PIDleftSetpoint = RamseteControl.getEntry("Left setpoint");
    var leftVel = RamseteControl.getEntry("Left Velocity");
    var PIDrightSetpoint = RamseteControl.getEntry("Right setpoint");
    var rightVel = RamseteControl.getEntry("Right Velocity");

    //original values: 0.2,0.5
    RamseteController disabled = new RamseteController(2, 0.7);
    PIDController leftController = new PIDController(10, 0, 0);
    PIDController rightController = new PIDController(10, 0, 0);
    disabled.setEnabled(false);
    
    return Commands.sequence(
      new RamseteCommand(
        trajectory, 
        drivetrain::getPose,
        disabled,
        new SimpleMotorFeedforward(0.15, 2, 1.5),//voltages here, arbitrary numbers here for now
        Settings.Drivetrain.KINEMATICS, 
        drivetrain::getWheelSpeeds, 
        leftController,
        rightController,
        (left, right) -> {
          if (displayStats == false) {
            drivetrain.voltageDrive(left, right);
            return;
          }
          drivetrain.displayPath(trajectory);

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
      )
      .andThen(
        () -> drivetrain.voltageDrive(0, 0)
      )
    );
  }

  public static CommandBase PIDtest(Drivetrain drivetrain, Limelight limelight) {
    // Only works with numbers around 3 meters, underneath that must have multiples of .45
    return Commands.sequence(
      new AutonDriveDistance(drivetrain, limelight, 4.5 , false)
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}