// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.sentinels.robot.commands.autonomous.Driving;

import com.sentinels.robot.commands.autonomous.Driving.SeperateDrive.AutonDriveDistance;
import com.sentinels.robot.commands.autonomous.Driving.SeperateDrive.AutonTurn;
import com.sentinels.robot.subsystems.drive.Drivetrain;
import com.sentinels.robot.subsystems.odometry.IMU;
import com.sentinels.robot.subsystems.vision.Limelight;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Seperate Drive is where it turns and then drives a given distance forward


// this may get deleted

public class AutonSeperateDrive extends SequentialCommandGroup {
  /** Creates a new AutonSeperateDrive. */
  public AutonSeperateDrive(Drivetrain drivetrain, Limelight limelight, IMU imu, double setpoint, boolean bool) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutonDriveDistance(drivetrain, limelight, setpoint, bool),
      new AutonTurn(drivetrain, limelight, imu),
      new AutonDriveDistance(drivetrain, limelight)
    );
  }
}
