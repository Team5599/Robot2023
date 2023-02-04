/***************************************************************
                The Sentinels - FRC Team 5599
        Benjamin N. Cardozo High School Robotics Team

    This work is licensed under the terms of the MIT license.
    Copyright (c) 2023 The Sentinels. All rights reserved.
***************************************************************/

/**
 * Code to allow the robot to move.
 * 
 */

package frc.robot.subsystems.drive;

import frc.robot.constants.Motors;
import frc.robot.constants.Ports;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  Spark motorBL = new Spark(Ports.Drivetrain.BACKLEFT);
  Spark motorFL = new Spark(Ports.Drivetrain.FRONTLEFT);
  Spark motorBR = new Spark(Ports.Drivetrain.BACKRIGHT);
  Spark motorFR = new Spark(Ports.Drivetrain.FRONTRIGHT);

  /** Creates a new ExampleSubsystem. */
  public Drivetrain() {

    //invert the one of the sides so that they rotate together
    motorBL.setInverted(true);
    motorFL.setInverted(true);

  }
  public void setSpeed(double leftSpeed, double rightSpeed){
    motorFL.set(leftSpeed);
    motorBL.set(leftSpeed);
    motorFR.set(rightSpeed);
    motorBR.set(rightSpeed);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
